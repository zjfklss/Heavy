#define GL_GLEXT_PROTOTYPES
#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GL/glcorearb.h>
#include "camera_planes.hpp"
#include <algorithm>
#include <cctype>
#include <cstddef>
#include <iostream>
#include <stdexcept>

namespace rm_sim::camera_detail {
namespace {
const EGLint context_attributes[]={EGL_CONTEXT_MAJOR_VERSION,3,EGL_CONTEXT_MINOR_VERSION,3,
  EGL_CONTEXT_OPENGL_PROFILE_MASK,EGL_CONTEXT_OPENGL_CORE_PROFILE_BIT,EGL_NONE};
const EGLint surface_attributes[]={EGL_WIDTH,1,EGL_HEIGHT,1,EGL_NONE};
class Device {
 public:
  EGLDisplay display=EGL_NO_DISPLAY;EGLConfig config{};std::string name;
  Device() {
    auto query=reinterpret_cast<PFNEGLQUERYDEVICESEXTPROC>(eglGetProcAddress("eglQueryDevicesEXT"));
    auto platform=reinterpret_cast<PFNEGLGETPLATFORMDISPLAYEXTPROC>(eglGetProcAddress("eglGetPlatformDisplayEXT"));
    if(query && platform) {
      EGLDeviceEXT devices[32];EGLint count=0;
      if(query(32,devices,&count))for(int i=0;i<count;++i)
        if(select(platform(EGL_PLATFORM_DEVICE_EXT,devices[i],nullptr)))return;
    }
    if(select(eglGetDisplay(EGL_DEFAULT_DISPLAY)))return;
    throw std::runtime_error("No hardware EGL/OpenGL 3.3 device available");
  }
  ~Device(){if(display!=EGL_NO_DISPLAY)eglTerminate(display);}
 private:
  bool select(EGLDisplay candidate) {
    EGLint major=0,minor=0;
    if(candidate==EGL_NO_DISPLAY || !eglInitialize(candidate,&major,&minor))return false;
    const EGLint attributes[]={EGL_SURFACE_TYPE,EGL_PBUFFER_BIT,EGL_RENDERABLE_TYPE,EGL_OPENGL_BIT,
      EGL_RED_SIZE,8,EGL_GREEN_SIZE,8,EGL_BLUE_SIZE,8,EGL_NONE};
    EGLConfig chosen{};EGLint count=0;
    EGLContext context=EGL_NO_CONTEXT;EGLSurface surface=EGL_NO_SURFACE;std::string renderer;
    if(eglBindAPI(EGL_OPENGL_API) && eglChooseConfig(candidate,attributes,&chosen,1,&count) && count) {
      surface=eglCreatePbufferSurface(candidate,chosen,surface_attributes);
      context=eglCreateContext(candidate,chosen,EGL_NO_CONTEXT,context_attributes);
      if(surface!=EGL_NO_SURFACE && context!=EGL_NO_CONTEXT && eglMakeCurrent(candidate,surface,surface,context)) {
        auto value=glGetString(GL_RENDERER);if(value)renderer=reinterpret_cast<const char*>(value);
      }
    }
    eglMakeCurrent(candidate,EGL_NO_SURFACE,EGL_NO_SURFACE,EGL_NO_CONTEXT);
    if(context!=EGL_NO_CONTEXT)eglDestroyContext(candidate,context);
    if(surface!=EGL_NO_SURFACE)eglDestroySurface(candidate,surface);
    auto lower=renderer;std::transform(lower.begin(),lower.end(),lower.begin(),[](unsigned char c){return std::tolower(c);});
    const bool software=lower.find("llvmpipe")!=std::string::npos || lower.find("softpipe")!=std::string::npos ||
      lower.find("software")!=std::string::npos || lower.find("swrast")!=std::string::npos;
    if(renderer.empty() || software){eglTerminate(candidate);return false;}
    display=candidate;config=chosen;name=std::move(renderer);
    std::cerr<<"[camera] hardware GPU: "<<name<<"; native-resolution framebuffer\n";
    return true;
  }
};
Device& device(){static Device value;return value;}
GLuint shader(GLenum type,const char* source) {
  GLuint result=glCreateShader(type);glShaderSource(result,1,&source,nullptr);glCompileShader(result);
  GLint ok=0;glGetShaderiv(result,GL_COMPILE_STATUS,&ok);
  if(!ok) {
    char log[4096]{};glGetShaderInfoLog(result,sizeof(log),nullptr,log);glDeleteShader(result);
    throw std::runtime_error(std::string("GPU camera shader: ")+log);
  }
  return result;
}
constexpr const char* vertex_source=R"GLSL(#version 330 core
layout(location=0) in vec3 camera;
layout(location=1) in vec2 coordinates;
layout(location=2) in vec3 color;
layout(location=3) in float kind;
layout(location=4) in float marking;
layout(location=5) in float cap_ratio;
uniform vec4 intrinsics; // fx,fy,cx,cy
uniform vec2 dimensions;
out vec2 uv;
flat out vec3 ink;
flat out int texture_kind;
flat out int layer;
flat out float radius;
void main() {
  // Optical +Y is image-down. Map it to framebuffer row-up so readback row 0 is image row 0.
  vec2 pixel=2.0*(intrinsics.xy*camera.xy+intrinsics.zw*camera.z)/dimensions-vec2(camera.z);
  gl_Position=vec4(pixel,camera.z-0.02,camera.z);
  uv=coordinates;ink=color;texture_kind=int(kind);layer=int(marking);radius=cap_ratio;
})GLSL";
constexpr const char* fragment_source=R"GLSL(#version 330 core
in vec2 uv;
flat in vec3 ink;
flat in int texture_kind;
flat in int layer;
flat in float radius;
uniform sampler2DArray markings;
out vec4 output_color;
void main() {
  if(texture_kind==2 && dot(uv,uv)>1.0)discard;
  if(texture_kind==4) {
    float cap=max(0.0,abs(uv.y)-(1.0-radius))/radius;
    if(uv.x*uv.x+cap*cap>1.0)discard;
  }
  vec3 color=ink;
  if(texture_kind==1) {
    float mask=texture(markings,vec3((uv.x+1.0)*0.5,(1.0-uv.y)*0.5,float(layer))).r;
    color=vec3(mix(12.0/255.0,240.0/255.0,mask));
  }
  output_color=vec4(color,1.0);
})GLSL";
struct Vertex {float position[3],uv[2],color[3],kind,marking,cap_ratio;};
class Renderer {
 public:
  Renderer():device_(device()) {
    try {
      ensure(eglBindAPI(EGL_OPENGL_API),"Cannot bind EGL OpenGL API");
      surface_=eglCreatePbufferSurface(device_.display,device_.config,surface_attributes);
      context_=eglCreateContext(device_.display,device_.config,EGL_NO_CONTEXT,context_attributes);
      ensure(context_!=EGL_NO_CONTEXT && surface_!=EGL_NO_SURFACE &&
        eglMakeCurrent(device_.display,surface_,surface_,context_),"Cannot create GPU camera context");
      GLuint vertex=shader(GL_VERTEX_SHADER,vertex_source),fragment=0;
      try {fragment=shader(GL_FRAGMENT_SHADER,fragment_source);}
      catch(...){glDeleteShader(vertex);throw;}
      program_=glCreateProgram();glAttachShader(program_,vertex);glAttachShader(program_,fragment);glLinkProgram(program_);
      glDeleteShader(vertex);glDeleteShader(fragment);
      GLint linked=0;glGetProgramiv(program_,GL_LINK_STATUS,&linked);ensure(linked,"Cannot link GPU camera shaders");
      glGenVertexArrays(1,&vao_);glBindVertexArray(vao_);glGenBuffers(1,&vbo_);glBindBuffer(GL_ARRAY_BUFFER,vbo_);
      const int sizes[]={3,2,3,1,1,1};const size_t offsets[]={offsetof(Vertex,position),offsetof(Vertex,uv),
        offsetof(Vertex,color),offsetof(Vertex,kind),offsetof(Vertex,marking),offsetof(Vertex,cap_ratio)};
      for(GLuint i=0;i<6;++i){glEnableVertexAttribArray(i);glVertexAttribPointer(i,sizes[i],GL_FLOAT,GL_FALSE,sizeof(Vertex),reinterpret_cast<void*>(offsets[i]));}
      glGenTextures(1,&atlas_);glBindTexture(GL_TEXTURE_2D_ARRAY,atlas_);
      std::vector<uint8_t> pixels(6*ArmorMarking::width*ArmorMarking::height);int layer=0;
      for(const auto* id:{"1","2","3","4","sentry","outpost"}) {
        const auto& mask=armor_marking(id);
        for(int y=0;y<ArmorMarking::height;++y)for(int x=0;x<ArmorMarking::width;++x)
          pixels[(layer*ArmorMarking::height+y)*ArmorMarking::width+x]=mask.ink(x,y)?255:0;
        ++layer;
      }
      glTexImage3D(GL_TEXTURE_2D_ARRAY,0,GL_R8,ArmorMarking::width,ArmorMarking::height,6,0,GL_RED,GL_UNSIGNED_BYTE,pixels.data());
      // Filter glyphs at their native pixel footprint; MSAA alone only smooths polygon edges.
      glGenerateMipmap(GL_TEXTURE_2D_ARRAY);
      glTexParameteri(GL_TEXTURE_2D_ARRAY,GL_TEXTURE_MIN_FILTER,GL_LINEAR_MIPMAP_LINEAR);glTexParameteri(GL_TEXTURE_2D_ARRAY,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
      glTexParameteri(GL_TEXTURE_2D_ARRAY,GL_TEXTURE_WRAP_S,GL_CLAMP_TO_EDGE);glTexParameteri(GL_TEXTURE_2D_ARRAY,GL_TEXTURE_WRAP_T,GL_CLAMP_TO_EDGE);
      glGetIntegerv(GL_MAX_SAMPLES,&samples_);samples_=std::min(4,samples_);
      glDisable(GL_DITHER);glDisable(GL_CULL_FACE);glDisable(GL_BLEND);glDisable(GL_FRAMEBUFFER_SRGB);
      glEnable(GL_DEPTH_TEST);glDepthFunc(GL_LEQUAL);glPixelStorei(GL_PACK_ALIGNMENT,1);
      ensure(glGetError()==GL_NO_ERROR,"GPU camera initialization failed");
    }catch(...){release();throw;}
  }
  ~Renderer(){release();}
  std::shared_ptr<Image> render(const Snapshot& snapshot,const Pose& pose,const CameraCalibration& k,const ImageConfig& config) {
    resize(k.width,k.height);vertices_.clear();
    planes(snapshot,pose,config,[&](Vec3 c,Vec3 right,Vec3 up,double hw,double hh,Color color,int kind,const ArmorMarking* marking){
      int layer=0;if(marking)for(const auto* id:{"1","2","3","4","sentry","outpost"}){if(marking->id==id)break;++layer;}
      const auto center=pose.inverse(c),r=pose.rotation.inverse(right)*hw,u=pose.rotation.inverse(up)*hh;
      for(const auto& uv:std::array<std::array<int,2>,6>{{{{-1,-1}},{{1,-1}},{{1,1}},{{-1,-1}},{{1,1}},{{-1,1}}}}) {
        auto p=center+r*uv[0]+u*uv[1];
        vertices_.push_back({{float(p.x),float(p.y),float(p.z)},{float(uv[0]),float(uv[1])},
          {color[2]/255.f,color[1]/255.f,color[0]/255.f},float(kind),float(layer),float(hw/hh)});
      }
    });
    glBindFramebuffer(GL_FRAMEBUFFER,multisample_);glViewport(0,0,width_,height_);
    glClearColor(28/255.f,28/255.f,28/255.f,1);glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
    glUseProgram(program_);glBindVertexArray(vao_);glBindBuffer(GL_ARRAY_BUFFER,vbo_);
    glBufferData(GL_ARRAY_BUFFER,GLsizeiptr(vertices_.size()*sizeof(Vertex)),vertices_.data(),GL_STREAM_DRAW);
    glUniform4f(glGetUniformLocation(program_,"intrinsics"),float(k.fx),float(k.fy),float(k.cx),float(k.cy));
    glUniform2f(glGetUniformLocation(program_,"dimensions"),float(width_),float(height_));
    glActiveTexture(GL_TEXTURE0);glBindTexture(GL_TEXTURE_2D_ARRAY,atlas_);glUniform1i(glGetUniformLocation(program_,"markings"),0);
    glDrawArrays(GL_TRIANGLES,0,GLsizei(vertices_.size()));
    glBindFramebuffer(GL_READ_FRAMEBUFFER,multisample_);glBindFramebuffer(GL_DRAW_FRAMEBUFFER,resolved_);
    glBlitFramebuffer(0,0,width_,height_,0,0,width_,height_,GL_COLOR_BUFFER_BIT,GL_NEAREST);
    glBindFramebuffer(GL_READ_FRAMEBUFFER,resolved_);glReadBuffer(GL_COLOR_ATTACHMENT0);
    auto image=std::make_shared<Image>();image->width=width_;image->height=height_;
    image->renderer="gpu";image->device=device_.name;image->bgr.resize(size_t(width_)*height_*3);
    glReadPixels(0,0,width_,height_,GL_BGR,GL_UNSIGNED_BYTE,image->bgr.data());
    ensure(glGetError()==GL_NO_ERROR,"GPU camera rendering/readback failed");return image;
  }
 private:
  Device& device_;EGLContext context_=EGL_NO_CONTEXT;EGLSurface surface_=EGL_NO_SURFACE;
  GLuint program_=0,vao_=0,vbo_=0,atlas_=0,multisample_=0,resolved_=0,color_=0,depth_=0,readback_=0;
  int width_=0,height_=0,samples_=1;std::vector<Vertex> vertices_;
  void clear_buffers(){
    glDeleteFramebuffers(1,&multisample_);glDeleteFramebuffers(1,&resolved_);
    glDeleteRenderbuffers(1,&color_);glDeleteRenderbuffers(1,&depth_);glDeleteRenderbuffers(1,&readback_);
    multisample_=resolved_=color_=depth_=readback_=0;
  }
  void resize(int width,int height) {
    if(width==width_ && height==height_)return;
    GLint limit=0;glGetIntegerv(GL_MAX_RENDERBUFFER_SIZE,&limit);ensure(width<=limit && height<=limit,"Camera size exceeds GPU framebuffer limit");
    clear_buffers();width_=width;height_=height;
    glGenFramebuffers(1,&multisample_);glBindFramebuffer(GL_FRAMEBUFFER,multisample_);
    glGenRenderbuffers(1,&color_);glBindRenderbuffer(GL_RENDERBUFFER,color_);
    glRenderbufferStorageMultisample(GL_RENDERBUFFER,samples_,GL_RGB8,width,height);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER,GL_COLOR_ATTACHMENT0,GL_RENDERBUFFER,color_);
    glGenRenderbuffers(1,&depth_);glBindRenderbuffer(GL_RENDERBUFFER,depth_);
    glRenderbufferStorageMultisample(GL_RENDERBUFFER,samples_,GL_DEPTH_COMPONENT24,width,height);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER,GL_DEPTH_ATTACHMENT,GL_RENDERBUFFER,depth_);
    ensure(glCheckFramebufferStatus(GL_FRAMEBUFFER)==GL_FRAMEBUFFER_COMPLETE,"GPU camera framebuffer incomplete");
    glGenFramebuffers(1,&resolved_);glBindFramebuffer(GL_FRAMEBUFFER,resolved_);
    glGenRenderbuffers(1,&readback_);glBindRenderbuffer(GL_RENDERBUFFER,readback_);glRenderbufferStorage(GL_RENDERBUFFER,GL_RGB8,width,height);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER,GL_COLOR_ATTACHMENT0,GL_RENDERBUFFER,readback_);
    ensure(glCheckFramebufferStatus(GL_FRAMEBUFFER)==GL_FRAMEBUFFER_COMPLETE,"GPU camera readback framebuffer incomplete");
  }
  void release() {
    if(context_!=EGL_NO_CONTEXT && surface_!=EGL_NO_SURFACE && eglMakeCurrent(device_.display,surface_,surface_,context_)) {
      clear_buffers();glDeleteTextures(1,&atlas_);glDeleteBuffers(1,&vbo_);glDeleteVertexArrays(1,&vao_);glDeleteProgram(program_);
      eglMakeCurrent(device_.display,EGL_NO_SURFACE,EGL_NO_SURFACE,EGL_NO_CONTEXT);
    }
    if(context_!=EGL_NO_CONTEXT)eglDestroyContext(device_.display,context_);
    if(surface_!=EGL_NO_SURFACE)eglDestroySurface(device_.display,surface_);
    context_=EGL_NO_CONTEXT;surface_=EGL_NO_SURFACE;
  }
};
}
std::shared_ptr<Image> render_gpu(const Snapshot& snapshot,const Pose& pose,const CameraCalibration& k,const ImageConfig& config) {
  // Contexts and framebuffer caches belong to the capture thread, never the Qt UI thread.
  thread_local Renderer renderer;return renderer.render(snapshot,pose,k,config);
}
} // namespace rm_sim::camera_detail
