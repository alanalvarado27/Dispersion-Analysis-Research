#define NOMINMAX
#define GLFW_INCLUDE_NONE

#include <iostream>
#include <limits>
#include <cmath>
#include <cwchar>
#include <vector>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <string>
#include <iterator>
#include <stdexcept>
#include <filesystem>
#include "offline_geometry.hpp"
#include "dimension_constraints.hpp"
#include <map>
#include "json.hpp"

#include <atlbase.h>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#if __has_include("solidworks_paths.hpp")
#include "solidworks_paths.hpp"
#else
#define SWORKS_TLB "C:/Program Files/SOLIDWORKS Corp/SOLIDWORKS (2)/sldworks.tlb"
#define SWCONST_TLB "C:/Program Files/SOLIDWORKS Corp/SOLIDWORKS (2)/swconst.tlb"
#endif
#import SWORKS_TLB raw_interfaces_only, raw_native_types, no_namespace, named_guids
#import SWCONST_TLB raw_interfaces_only, raw_native_types, no_namespace, named_guids

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"



#pragma comment(lib, "ole32.lib")
#pragma comment(lib, "oleaut32.lib")

namespace
{
    using json = nlohmann::json;
    using sketch::Point;
    using sketch::Document;
    using sketch::Kind;
    using Point2D = Point;
    int kImageWidth = 0, kImageHeight = 0;
    double viewX=0,viewY=0,viewW=1,viewH=1;
    float ndcX(double x){return float(2*(x-viewX)/viewW-1);}
    float ndcY(double y){return float(1-2*(y-viewY)/viewH);}
    struct ClickState {
        std::vector<Point> points;
        Document* document = nullptr;
        Document original, beforeDrag;
        const sketch::DimensionSystem* dimensions=nullptr;
        bool geometryOnly=false, protectTopology=false;
        bool constrained()const{return dimensions&&!dimensions->equations.empty();}
        int selectedPoint = -1;
        bool dragging = false, accepted = false;
    };
    struct Viewport { double x, y, w, h; };
    Viewport imageViewport(GLFWwindow* window) {
        int w,h; glfwGetWindowSize(window,&w,&h);
        double scale=std::min(double(w)/viewW,double(h)/viewH);
        return {(w-viewW*scale)/2,(h-viewH*scale)/2,viewW*scale,viewH*scale};
    }
    bool cursorImage(GLFWwindow* window, Point& p) {
        double x,y; glfwGetCursorPos(window,&x,&y);
        auto v=imageViewport(window);
        if(v.w<=0 || v.h<=0) return false;
        p={viewX+(x-v.x)*viewW/v.w,viewY+(y-v.y)*viewH/v.h};
        return x>=v.x&&y>=v.y&&x<=v.x+v.w&&y<=v.y+v.h;
    }
    GLuint compileShader(GLenum type, const char* source)
    {
        const GLuint shader = glCreateShader(type);
        glShaderSource(shader, 1, &source, nullptr);
        glCompileShader(shader);

        GLint success = GL_FALSE;
        glGetShaderiv(shader, GL_COMPILE_STATUS, &success);

        if (success == GL_FALSE)
        {
            char message[1024]{};
            glGetShaderInfoLog(shader, sizeof(message), nullptr, message);
            std::cerr << "Shader compilation failed: " << message << '\n';
            glDeleteShader(shader);
            return 0;
        }

        return shader;
    }

    GLuint createProgram(const char* vertexSource, const char* fragmentSource)
    {
        const GLuint vertexShader = compileShader(GL_VERTEX_SHADER, vertexSource);
        const GLuint fragmentShader = compileShader(GL_FRAGMENT_SHADER, fragmentSource);

        if (vertexShader == 0 || fragmentShader == 0)
        {
            glDeleteShader(vertexShader);
            glDeleteShader(fragmentShader);
            return 0;
        }

        const GLuint program = glCreateProgram();
        glAttachShader(program, vertexShader);
        glAttachShader(program, fragmentShader);
        glLinkProgram(program);

        glDeleteShader(vertexShader);
        glDeleteShader(fragmentShader);

        GLint success = GL_FALSE;
        glGetProgramiv(program, GL_LINK_STATUS, &success);

        if (success == GL_FALSE)
        {
            char message[1024]{};
            glGetProgramInfoLog(program, sizeof(message), nullptr, message);
            std::cerr << "Shader linking failed: " << message << '\n';
            glDeleteProgram(program);
            return 0;
        }

        return program;
    }

    void mouseButtonCallback(GLFWwindow* window, int button, int action, int) {
        auto& state=*static_cast<ClickState*>(glfwGetWindowUserPointer(window));
        if(button==GLFW_MOUSE_BUTTON_LEFT && action==GLFW_RELEASE){
            if(state.dragging&&state.constrained()){
                try{sketch::solveDimensions(*state.document,*state.dimensions,state.selectedPoint);}
                catch(const std::exception& e){*state.document=state.beforeDrag;std::cout<<e.what()<<"\nDrag undone.\n";}
            }
            state.dragging=false;
        }
        if(action!=GLFW_PRESS) return;
        Point p; if(!cursorImage(window,p)) return;
        if(!state.document) {
            if(button!=GLFW_MOUSE_BUTTON_LEFT || state.points.size()>=2) return;
            if(!state.points.empty() && sketch::distance(p,state.points[0])<2) return;
            state.points.push_back(p);
            std::cout << "Selected (" << p.x << ", " << p.y << ") image pixels.\n";
            return;
        }
        auto& d=*state.document;
        double best=14*viewW/imageViewport(window).w;
        if(button==GLFW_MOUSE_BUTTON_LEFT) {
            int n=-1;
            for(size_t i=0;i<d.points.size();++i) {
                double distance=sketch::distance(p,d.points[i]);
                if(distance<best){best=distance;n=int(i);}
            }
            state.selectedPoint=n;state.dragging=n>=0;
            if(n>=0)state.beforeDrag=d;
            if(n>=0) std::cout << "Selected " << d.ids[n] << " (" << d.points[n].x << ", " << d.points[n].y << ")\n";
        } else if(button==GLFW_MOUSE_BUTTON_RIGHT) {
            if(state.constrained()||state.protectTopology){std::cout<<"Splitting constrained sketches is disabled; edit the JSON topology.\n";return;}
            int ci=-1,ei=-1;Point q;
            for(size_t c=0;c<d.contours.size();++c)for(size_t i=0;i<d.contours[c].entities.size();++i){
                const auto& e=d.contours[c].entities[i];if(e.kind!=Kind::Line)continue;
                auto candidate=sketch::nearestOnEdge(p,d.points[e.a],d.points[e.b]);
                double distance=sketch::distance(p,candidate);
                if(distance<best){best=distance;ci=int(c);ei=int(i);q=candidate;}
            }
            if(ci>=0){sketch::splitLine(d,ci,ei,q);state.selectedPoint=-1;state.dragging=false;}
        }
    }
    void keyCallback(GLFWwindow* window,int key,int,int action,int) {
        if(action!=GLFW_PRESS)return;
        auto& state=*static_cast<ClickState*>(glfwGetWindowUserPointer(window));
        if(key==GLFW_KEY_ESCAPE)glfwSetWindowShouldClose(window,GLFW_TRUE);
        if(key==GLFW_KEY_R){
            state.points.clear();if(state.document)*state.document=state.original;
            state.selectedPoint=-1;state.dragging=false;
        }
        if(key==GLFW_KEY_DELETE&&state.document&&state.selectedPoint>=0){
            if(state.constrained()||state.protectTopology||!sketch::removeLinePoint(*state.document,state.selectedPoint))
                std::cout << "Delete is disabled for dimensioned sketches; otherwise select an unshared line vertex (at least 3 remain).\n";
            state.selectedPoint=-1;state.dragging=false;
        }
        if(key==GLFW_KEY_ENTER||key==GLFW_KEY_KP_ENTER){
            try{
                if(state.document){
                    if(state.constrained())sketch::solveDimensions(*state.document,*state.dimensions);
                    sketch::validate(*state.document,kImageWidth,kImageHeight,false);
                }
                else if(state.points.size()!=2)return;
                state.accepted=true;
            }catch(const std::exception& e){std::cerr << "Fix before accepting: " << e.what() << '\n';}
        }
    }
    void drawRedDot(GLuint colorProgram, GLuint dotVao, GLuint dotVbo,
                    double pixelX, double pixelY)
    {
        constexpr int segmentCount = 40;
        const float radiusPx = float(std::max(viewW,viewH)*0.003);

        std::vector<float> vertices;
        vertices.reserve((segmentCount + 2) * 2);

        const auto toNdcX = [](double x)
        {
            return ndcX(x);
        };

        const auto toNdcY = [](double y)
        {
            return ndcY(y);
        };

        vertices.push_back(toNdcX(pixelX));
        vertices.push_back(toNdcY(pixelY));

        for (int i = 0; i <= segmentCount; ++i)
        {
            const double angle = 2.0 * 3.14159265358979323846 * i / segmentCount;
            vertices.push_back(toNdcX(pixelX + radiusPx * std::cos(angle)));
            vertices.push_back(toNdcY(pixelY + radiusPx * std::sin(angle)));
        }

        glUseProgram(colorProgram);
        glBindVertexArray(dotVao);
        glBindBuffer(GL_ARRAY_BUFFER, dotVbo);
        glBufferData(
            GL_ARRAY_BUFFER,
            static_cast<GLsizeiptr>(vertices.size() * sizeof(float)),
            vertices.data(),
            GL_DYNAMIC_DRAW
        );
        glDrawArrays(GL_TRIANGLE_FAN, 0, segmentCount + 2);
    }

    bool showImage(const std::string& imagePath, ClickState& clickState)
    {
        viewX=0;viewY=0;viewW=kImageWidth;viewH=kImageHeight;
        if(clickState.document){
            double minX=0,minY=0,maxX=kImageWidth,maxY=kImageHeight;
            if(clickState.geometryOnly){minX=minY=std::numeric_limits<double>::infinity();maxX=maxY=-minX;}
            auto include=[&](Point p){minX=std::min(minX,p.x);minY=std::min(minY,p.y);maxX=std::max(maxX,p.x);maxY=std::max(maxY,p.y);};
            for(auto p:clickState.document->points)include(p);
            for(const auto& c:clickState.document->contours)for(const auto& e:c.entities){
                try{for(auto p:sketch::sample(*clickState.document,e))include(p);}catch(const std::exception&){}
            }
            double pad=0.08*std::max({maxX-minX,maxY-minY,1.0});
            viewX=minX-pad;viewY=minY-pad;viewW=maxX-minX+2*pad;viewH=maxY-minY+2*pad;
        }
        if (glfwInit() == GLFW_FALSE)
        {
            std::cerr << "GLFW initialization failed.\n";
            return false;
        }

        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);

        GLFWwindow* window = glfwCreateWindow(
            1100,
            750,
            clickState.geometryOnly ? "Dimensioned sketch - drag handles; dimensions enforced on release; Enter: accept" :
            clickState.document ? "Review - drag handles; right click line: split; Del: remove line vertex; Enter: accept" :
                               "Calibration - click two points, then Enter; R: reset; Esc: cancel",
            nullptr,
            nullptr
        );

        if (window == nullptr)
        {
            std::cerr << "OpenGL window creation failed.\n";
            glfwTerminate();
            return false;
        }

        glfwMakeContextCurrent(window);
        glfwSwapInterval(1);

        if (!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress)))
        {
            std::cerr << "GLAD initialization failed.\n";
            glfwDestroyWindow(window);
            glfwTerminate();
            return false;
        }

        glfwSetWindowUserPointer(window, &clickState);
        glfwSetMouseButtonCallback(window, mouseButtonCallback);
        glfwSetKeyCallback(window, keyCallback);

        int loadedWidth = 0;
        int loadedHeight = 0;
        int loadedChannels = 0;

        stbi_set_flip_vertically_on_load(true);
        unsigned char* imageData = stbi_load(
            imagePath.c_str(),
            &loadedWidth,
            &loadedHeight,
            &loadedChannels,
            3
        );

        if (imageData == nullptr)
        {
            std::cerr << "Could not load " << imagePath << "\n";
            glfwDestroyWindow(window);
            glfwTerminate();
            return false;
        }

        kImageWidth=loadedWidth; kImageHeight=loadedHeight;

        GLuint texture = 0;
        glGenTextures(1, &texture);
        glBindTexture(GL_TEXTURE_2D, texture);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glTexImage2D(
            GL_TEXTURE_2D,
            0,
            GL_RGB,
            loadedWidth,
            loadedHeight,
            0,
            GL_RGB,
            GL_UNSIGNED_BYTE,
            imageData
        );
        stbi_image_free(imageData);

        constexpr char imageVertexShader[] = R"(
            #version 330 core
            layout(location = 0) in vec2 position;
            layout(location = 1) in vec2 textureCoordinate;
            out vec2 uv;
            void main()
            {
                uv = textureCoordinate;
                gl_Position = vec4(position, 0.0, 1.0);
            }
        )";

        constexpr char imageFragmentShader[] = R"(
            #version 330 core
            in vec2 uv;
            out vec4 fragmentColor;
            uniform sampler2D imageTexture;
            void main()
            {
                fragmentColor = texture(imageTexture, uv);
            }
        )";

        constexpr char colorVertexShader[] = R"(
            #version 330 core
            layout(location = 0) in vec2 position;
            void main()
            {
                gl_Position = vec4(position, 0.0, 1.0);
            }
        )";

        constexpr char colorFragmentShader[] = R"(
            #version 330 core
            out vec4 fragmentColor;
            uniform vec3 color;
            void main() { fragmentColor = vec4(color, 1.0); }
        )";

        const GLuint imageProgram = createProgram(imageVertexShader, imageFragmentShader);
        const GLuint colorProgram = createProgram(colorVertexShader, colorFragmentShader);

        if (imageProgram == 0 || colorProgram == 0)
        {
            glDeleteTextures(1, &texture);
            glfwDestroyWindow(window);
            glfwTerminate();
            return false;
        }

        const float quadVertices[] = {
            // position     // texture coordinates
            ndcX(0), ndcY(kImageHeight), 0.0f, 0.0f,
             ndcX(kImageWidth), ndcY(kImageHeight), 1.0f, 0.0f,
             ndcX(kImageWidth), ndcY(0), 1.0f, 1.0f,
            ndcX(0), ndcY(0), 0.0f, 1.0f
        };

        const unsigned int quadIndices[] = {0, 1, 2, 2, 3, 0};

        GLuint quadVao = 0;
        GLuint quadVbo = 0;
        GLuint quadEbo = 0;
        glGenVertexArrays(1, &quadVao);
        glGenBuffers(1, &quadVbo);
        glGenBuffers(1, &quadEbo);

        glBindVertexArray(quadVao);
        glBindBuffer(GL_ARRAY_BUFFER, quadVbo);
        glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), quadVertices, GL_STATIC_DRAW);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, quadEbo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(quadIndices), quadIndices, GL_STATIC_DRAW);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), nullptr);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(
            1,
            2,
            GL_FLOAT,
            GL_FALSE,
            4 * sizeof(float),
            reinterpret_cast<void*>(2 * sizeof(float))
        );
        glEnableVertexAttribArray(1);

        GLuint dotVao = 0;
        GLuint dotVbo = 0;
        glGenVertexArrays(1, &dotVao);
        glGenBuffers(1, &dotVbo);
        glBindVertexArray(dotVao);
        glBindBuffer(GL_ARRAY_BUFFER, dotVbo);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), nullptr);
        glEnableVertexAttribArray(0);

        std::cout << (clickState.document ? "Review the contour; Enter accepts.\n" :
            "Click two distinct points, then press Enter.\n");
        std::cout << "Press R to reset or Escape to cancel.\n";


        while (glfwWindowShouldClose(window) == GLFW_FALSE && !clickState.accepted)
        {
            glfwPollEvents();

            int fw,fh,ww,wh;
            glfwGetFramebufferSize(window,&fw,&fh);glfwGetWindowSize(window,&ww,&wh);
            if(ww<=0 || wh<=0 || fw<=0 || fh<=0) {glfwWaitEventsTimeout(0.05);continue;}
            auto v=imageViewport(window);
            glViewport(int(v.x*fw/ww),int((wh-v.y-v.h)*fh/wh),int(v.w*fw/ww),int(v.h*fh/wh));
            if(clickState.dragging && clickState.document) {
                Point p;
                if(cursorImage(window,p)) sketch::movePoint(*clickState.document,clickState.selectedPoint,p);
            }
            glClearColor(0.12f, 0.12f, 0.12f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT);

            glUseProgram(imageProgram);
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, texture);
            glUniform1i(glGetUniformLocation(imageProgram, "imageTexture"), 0);
            glBindVertexArray(quadVao);
            if(!clickState.geometryOnly)glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, nullptr);

            glUseProgram(colorProgram);
            glUniform3f(glGetUniformLocation(colorProgram,"color"),1.0f,0.1f,0.1f);
            for(const auto& point:clickState.points)
                drawRedDot(colorProgram,dotVao,dotVbo,point.x,point.y);
            if(clickState.document) {
                const auto& d=*clickState.document;
                auto drawPath=[&](const std::vector<Point>& points,float r,float g,float b){
                    std::vector<float> vertices;
                    for(auto p:points){vertices.push_back(ndcX(p.x));vertices.push_back(ndcY(p.y));}
                    glUseProgram(colorProgram);glUniform3f(glGetUniformLocation(colorProgram,"color"),r,g,b);
                    glBindVertexArray(dotVao);glBindBuffer(GL_ARRAY_BUFFER,dotVbo);
                    glBufferData(GL_ARRAY_BUFFER,vertices.size()*sizeof(float),vertices.data(),GL_DYNAMIC_DRAW);
                    glDrawArrays(GL_LINE_STRIP,0,GLsizei(points.size()));
                };
                for(const auto& contour:d.contours)for(const auto& e:contour.entities){
                    try{drawPath(sketch::sample(d,e),0,1,0.3f);}
                    catch(const std::exception&){drawPath({d.points[e.a],d.points[e.b]},1,0,0);}
                    if(e.kind==Kind::Circle)drawPath({d.points[e.a],d.points[e.b]},0.3f,0.6f,1);
                }
                for(size_t i=0;i<d.points.size();++i){
                    bool selected=int(i)==clickState.selectedPoint;
                    glUseProgram(colorProgram);glUniform3f(glGetUniformLocation(colorProgram,"color"),1,selected?1.0f:0.4f,selected?1.0f:0);
                    drawRedDot(colorProgram,dotVao,dotVbo,d.points[i].x,d.points[i].y);
                }
            }
            glfwSwapBuffers(window);
        }

        glDeleteBuffers(1, &dotVbo);
        glDeleteVertexArrays(1, &dotVao);
        glDeleteBuffers(1, &quadEbo);
        glDeleteBuffers(1, &quadVbo);
        glDeleteVertexArrays(1, &quadVao);
        glDeleteProgram(colorProgram);
        glDeleteProgram(imageProgram);
        glDeleteTextures(1, &texture);
        glfwDestroyWindow(window);
        glfwTerminate();

        return clickState.accepted;
    }

    double positiveNumber(const char* prompt) {
        for(;;) {
            std::cout << prompt;
            std::string line;
            if(!std::getline(std::cin,line)) throw std::runtime_error("Input canceled.");
            std::istringstream in(line); double value; std::string extra;
            if(in>>value && !(in>>extra) && std::isfinite(value) && value>0) return value;
            std::cout << "Enter a finite, positive number.\n";
        }
    }
    void writeJson(const std::filesystem::path& path,const json& value) {
        std::ofstream out(path);
        if(!out || !(out<<value.dump(2)<<'\n')) throw std::runtime_error("Cannot write "+path.string());
    }
    Document parseSketch(const json& j) {
        if(j.at("schema").get<std::string>()!="chat_sketch_v1")throw std::runtime_error("Expected schema chat_sketch_v1.");
        if(j.at("coordinate_system").get<std::string>()!="image_pixels_top_left")
            throw std::runtime_error("Expected image_pixels_top_left coordinates.");
        if(j.at("image").at("width").get<int>()!=kImageWidth||j.at("image").at("height").get<int>()!=kImageHeight)
            throw std::runtime_error("JSON image dimensions do not match this image. Use the original unresized image.");
        Document d;std::map<std::string,int> ids;
        const auto& points=j.at("points");
        if(!points.is_array()||points.empty()||points.size()>6000)throw std::runtime_error("Invalid point count.");
        for(const auto& p:points){
            std::string id=p.at("id").get<std::string>();
            if(id.empty()||ids.count(id))throw std::runtime_error("Empty or duplicate point ID.");
            ids[id]=int(d.points.size());d.ids.push_back(id);
            double x=p.at("x").get<double>(),y=p.at("y").get<double>();
            if(!std::isfinite(x)||!std::isfinite(y)||std::abs(x)>1e9||std::abs(y)>1e9)
                throw std::runtime_error("Invalid image point: "+id);
            d.points.push_back({x,y});
        }
        auto ref=[&](const json& e,const char* name){
            auto id=e.at(name).get<std::string>();auto p=ids.find(id);
            if(p==ids.end())throw std::runtime_error("Unknown point ID: "+id);
            return p->second;
        };
        const auto& contours=j.at("contours");
        if(!contours.is_array()||contours.empty()||contours.size()>128)throw std::runtime_error("Invalid contour count.");
        size_t count=0;
        for(const auto& c:contours){
            sketch::Contour out;out.name=c.at("name").get<std::string>();out.role=c.value("role",std::string());
            if(!c.at("entities").is_array()||c.at("entities").empty())throw std::runtime_error("Empty contour.");
            for(const auto& e:c.at("entities")){
                if(++count>6000)throw std::runtime_error("Too many sketch entities.");
                auto type=e.at("type").get<std::string>();sketch::Entity ent;
                if(type=="line"){ent={Kind::Line,ref(e,"start"),ref(e,"end"),-1};}
                else if(type=="arc"){ent={Kind::Arc,ref(e,"start"),ref(e,"end"),ref(e,"through")};}
                else if(type=="circle"){ent={Kind::Circle,ref(e,"center"),ref(e,"rim"),-1};}
                else throw std::runtime_error("Unsupported entity type: "+type);
                out.entities.push_back(ent);
            }
            d.contours.push_back(out);
        }
        // Reject unused nodes and shared circle handles so center dragging is unambiguous.
        std::vector<int> used(d.points.size(),0);
        for(const auto& c:d.contours)for(const auto& e:c.entities){++used[e.a];++used[e.b];if(e.c>=0)++used[e.c];}
        for(int n:used)if(!n)throw std::runtime_error("Remove unused control points from JSON.");
        for(const auto& c:d.contours)for(const auto& e:c.entities)
            if(e.kind==Kind::Circle&&(used[e.a]!=1||used[e.b]!=1))throw std::runtime_error("Circle handles must have unique IDs.");
        return d;
    }
    json serializeSketch(const Document& d,const std::string& imagePath){
        json points=json::array(),contours=json::array();
        for(size_t i=0;i<d.points.size();++i)points.push_back({{"id",d.ids[i]},{"x",d.points[i].x},{"y",d.points[i].y}});
        for(const auto& c:d.contours){
            json entities=json::array();
            for(const auto& e:c.entities){
                if(e.kind==Kind::Circle)entities.push_back({{"type","circle"},{"center",d.ids[e.a]},{"rim",d.ids[e.b]}});
                else{
                    json entity={{"type",e.kind==Kind::Line?"line":"arc"},{"start",d.ids[e.a]},{"end",d.ids[e.b]}};
                    if(e.kind==Kind::Arc)entity["through"]=d.ids[e.c];
                    entities.push_back(entity);
                }
            }
            contours.push_back({{"name",c.name},{"role",c.role.empty()?"auto":c.role},{"entities",entities}});
        }
        return {{"schema","chat_sketch_v1"},{"coordinate_system","image_pixels_top_left"},
                {"image",{{"file",std::filesystem::path(imagePath).filename().string()},{"width",kImageWidth},{"height",kImageHeight}}},
                {"points",points},{"contours",contours}};
    }
    void checked(HRESULT hr,const char* operation) {
        if(FAILED(hr)) {
            std::ostringstream text;text<<operation<<" failed, HRESULT 0x"<<std::hex<<static_cast<unsigned long>(hr);
            throw std::runtime_error(text.str());
        }
    }
    struct ComSession {
        ComSession(){checked(CoInitializeEx(nullptr,COINIT_APARTMENTTHREADED),"COM initialization");}
        ~ComSession(){CoUninitialize();}
    };
    void createSolidWorksPart(const Document& drawing,Point origin,double scale,double depthMm,
                             const std::string& plane) {
        ComSession com;
        CComPtr<ISldWorks> app;
        checked(app.CoCreateInstance(__uuidof(SldWorks),nullptr,CLSCTX_LOCAL_SERVER),"Start SOLIDWORKS");
        checked(app->put_Visible(VARIANT_TRUE),"Show SOLIDWORKS");
        CComBSTR partTemplate;
        checked(app->GetUserPreferenceStringValue(swDefaultTemplatePart,&partTemplate),"Get part template");
        if(partTemplate.Length()==0) throw std::runtime_error("Set Tools > Options > Default Templates > Part in SOLIDWORKS.");
        CComPtr<IModelDoc2> doc;
        checked(app->INewDocument2(partTemplate,0,0,0,&doc),"Create new part");
        if(!doc) throw std::runtime_error("The default part template could not create a document.");
        CComPtr<IModelDocExtension> extension;CComPtr<ISketchManager> manager;
        checked(doc->get_Extension(&extension),"Get document extension");
        checked(doc->get_SketchManager(&manager),"Get sketch manager");
        if(!extension||!manager) throw std::runtime_error("Missing SOLIDWORKS interfaces.");
        checked(doc->ClearSelection2(VARIANT_TRUE),"Clear selection");
        VARIANT_BOOL selected=VARIANT_FALSE;
        checked(extension->SelectByID2(CComBSTR(plane.c_str()),CComBSTR(L"PLANE"),0,0,0,
            VARIANT_FALSE,0,nullptr,swSelectOptionDefault,&selected),"Select sketch plane");
        // SOLIDWORKS has returned both 1 and -1 for true in the original setup.
        if(selected==VARIANT_FALSE) throw std::runtime_error("Plane not found. Pass its exact name as the last argument.");
        checked(manager->InsertSketch(VARIANT_TRUE),"Start sketch");
        checked(manager->put_AddToDB(VARIANT_TRUE),"Disable sketch inferencing");
        try {
            for(const auto& contour:drawing.contours)for(const auto& e:contour.entities){
                Point a=sketch::toMeters(drawing.points[e.a],origin,scale);
                Point b=sketch::toMeters(drawing.points[e.b],origin,scale);
                if(sketch::distance(a,b)<1e-7)throw std::runtime_error("A calibrated edge or radius is below 0.0001 mm.");
                CComPtr<ISketchSegment> segment;
                if(e.kind==Kind::Line)
                    checked(manager->CreateLine(a.x,a.y,0,b.x,b.y,0,&segment),"Create sketch line");
                else if(e.kind==Kind::Circle)
                    checked(manager->CreateCircleByRadius(a.x,a.y,0,sketch::distance(a,b),&segment),"Create sketch circle");
                else{
                    Point p=sketch::toMeters(drawing.points[e.c],origin,scale);
                    // SOLIDWORKS order is START, END, THROUGH (not start, through, end).
                    checked(manager->Create3PointArc(a.x,a.y,0,b.x,b.y,0,p.x,p.y,0,&segment),"Create sketch arc");
                }
                if(!segment)throw std::runtime_error("SOLIDWORKS rejected a sketch entity.");
            }
        } catch(...) {
            manager->put_AddToDB(VARIANT_FALSE);manager->InsertSketch(VARIANT_TRUE);throw;
        }
        checked(manager->put_AddToDB(VARIANT_FALSE),"Restore sketch inferencing");
        checked(manager->InsertSketch(VARIANT_TRUE),"Finish sketch");
        checked(doc->ClearSelection2(VARIANT_TRUE),"Clear line selections");
        // The new sketch is the last top-level ProfileFeature. Do not assume an English Sketch1 name.
        CComPtr<IFeature> feature,sketchFeature;
        checked(doc->IFirstFeature(&feature),"Get feature tree");
        while(feature) {
            CComBSTR type;checked(feature->GetTypeName2(&type),"Get feature type");
            if(type && std::wcscmp(type,L"ProfileFeature")==0) sketchFeature=feature;
            CComPtr<IFeature> next;checked(feature->IGetNextFeature(&next),"Get next feature");feature=next;
        }
        if(!sketchFeature) throw std::runtime_error("Could not locate the completed sketch; it remains in the new part.");
        selected=VARIANT_FALSE;
        checked(sketchFeature->Select2(VARIANT_FALSE,0,&selected),"Select completed sketch");
        if(selected==VARIANT_FALSE) throw std::runtime_error("Could not select the completed sketch.");
        CComPtr<IFeatureManager> features;checked(doc->get_FeatureManager(&features),"Get feature manager");
        if(!features) throw std::runtime_error("Feature manager is unavailable.");
        CComPtr<IFeature> extrusion;
        checked(features->FeatureExtrusion3(
            VARIANT_TRUE,VARIANT_FALSE,VARIANT_FALSE, // single-ended, no flip, normal direction
            swEndCondBlind,swEndCondBlind,depthMm/1000.0,0.0,
            VARIANT_FALSE,VARIANT_FALSE,VARIANT_FALSE,VARIANT_FALSE,0.0,0.0, // no draft
            VARIANT_FALSE,VARIANT_FALSE,VARIANT_FALSE,VARIANT_FALSE,
            VARIANT_TRUE,VARIANT_TRUE,VARIANT_TRUE, // merge, use feature scope, auto select
            swStartSketchPlane,0.0,VARIANT_FALSE,&extrusion),"Extrude sketch");
        if(!extrusion) throw std::runtime_error("Extrusion failed. The sketch is left in the new part for inspection.");
        doc->ViewZoomtofit2();doc->GraphicsRedraw2();
        std::cout << "Created sketch and " << depthMm << " mm extrusion. Save the new part using Ctrl+S.\n";
    }
}
int main(int argc,char** argv) {
    try {
        if(argc<3){
            std::cout << "Usage: sketch_to_solidworks.exe image.png sketch.json [--plane \"Front Plane\"] [--preview-only]\n"
                         "Offline workflow: upload image + CHATGPT_PROMPT.txt in chat, download JSON, run this program.\n"
                         "No Ollama, API account, or automatic image upload is used.\n";
            return argc==1?0:1;
        }
        std::string imagePath=argv[1],sketchFile=argv[2],plane="Front Plane";bool previewOnly=false;
        for(int i=3;i<argc;++i){
            std::string arg=argv[i];
            if(arg=="--preview-only")previewOnly=true;
            else if(arg=="--plane"&&i+1<argc)plane=argv[++i];
            else throw std::runtime_error("Unknown or incomplete option: "+arg);
        }
        int channels;
        if(!stbi_info(imagePath.c_str(),&kImageWidth,&kImageHeight,&channels))throw std::runtime_error("Cannot read image. Use a JPG or PNG path.");
        if(kImageWidth<2||kImageHeight<2||static_cast<long long>(kImageWidth)*kImageHeight>25000000)
            throw std::runtime_error("Use an image between 2 pixels and 25 megapixels.");
        if(std::filesystem::file_size(sketchFile)>8*1024*1024)throw std::runtime_error("Sketch JSON exceeds 8 MB.");
        json source;{std::ifstream in(sketchFile);if(!in)throw std::runtime_error("Cannot read sketch JSON.");in>>source;}
        Document drawing=parseSketch(source);
        if(source.contains("notes"))std::cout << "Sketch notes: " << source["notes"].dump() << '\n';
        try{sketch::validate(drawing,kImageWidth,kImageHeight,false);}
        catch(const std::exception& e){std::cout << "Fix before accepting: " << e.what() << '\n';}
        if(source.contains("material"))std::cout<<"Material interpretation: "<<source["material"].dump()<<'\n';
        std::map<std::string,double> values;
        auto dims=source.value("dimensions",json::array());
        if(!dims.is_array()||dims.size()>128)throw std::runtime_error("Invalid dimensions array.");
        for(const auto& dim:dims){
            std::string name=dim.at("name").get<std::string>();
            if(name.empty())throw std::runtime_error("Dimension name cannot be empty.");
            auto eq=sketch::readEquation(drawing,dim);
            if(!sketch::isDimension(eq.type))throw std::runtime_error("Unsupported dimension type.");
            if(values.count(name))continue;
            double def=dim.contains("value_mm")&&!dim["value_mm"].is_null()?dim["value_mm"].get<double>():0;
            if(!std::isfinite(def)||def<0)throw std::runtime_error("Invalid default dimension value.");
            for(;;){
                std::cout<<"Dimension "<<name<<" ("<<eq.type<<") in mm";
                if(def>0)std::cout<<" [Enter = "<<def<<"]";
                std::cout<<": ";std::string line;if(!std::getline(std::cin,line))throw std::runtime_error("Input canceled.");
                if(line.empty()&&def>0){values[name]=def;break;}
                std::istringstream in(line);double v;std::string extra;
                if(in>>v&&!(in>>extra)&&std::isfinite(v)&&v>0){values[name]=v;break;}
                std::cout<<"Enter a finite positive dimension in millimeters.\n";
            }
        }
        ClickState calibration;double scale=0,knownMm=0;Point origin;
        if(dims.empty()){
            if(!showImage(imagePath,calibration)){std::cout<<"Canceled.\n";return 0;}
            knownMm=positiveNumber("Real distance between selected points (mm): ");
            scale=knownMm/sketch::distance(calibration.points[0],calibration.points[1]);origin=calibration.points[0];
        }else{
            auto e=sketch::readEquation(drawing,dims[0]);double pixels=sketch::measured(drawing,e);
            if(pixels<0.05)throw std::runtime_error("First dimension has no measurable span in the image.");
            scale=values.at(dims[0].at("name").get<std::string>())/pixels;origin=drawing.points.front();
            std::cout<<"Scale established from "<<dims[0].at("name")<<"; two-point calibration skipped.\n";
        }
        if(!std::isfinite(scale)||scale<=0)throw std::runtime_error("Invalid calibration scale.");
        std::cout<<"Scale: "<<scale<<" mm/pixel. First review the traced geometry over the image.\n";
        ClickState preview;preview.document=&drawing;preview.original=drawing;preview.protectTopology=!dims.empty()||!source.value("constraints",json::array()).empty();
        if(!showImage(imagePath,preview)){std::cout<<"Canceled before creating a part.\n";return 0;}
        auto dimensionSystem=sketch::makeDimensions(drawing,source,values,scale);
        if(!dimensionSystem.equations.empty()){
            sketch::solveDimensions(drawing,dimensionSystem);
            sketch::validate(drawing,kImageWidth,kImageHeight,false);
            std::cout<<"Now review the resized geometry without the image. Named dimensions remain enforced on drag release.\n";
            ClickState finalReview;finalReview.document=&drawing;finalReview.original=drawing;
            finalReview.dimensions=&dimensionSystem;finalReview.geometryOnly=true;
            if(!showImage(imagePath,finalReview)){std::cout<<"Canceled.\n";return 0;}
            std::cout<<"Maximum constraint residual: "<<sketch::dimensionError(drawing,dimensionSystem)*scale<<" mm\n";
        }
        sketch::validate(drawing,kImageWidth,kImageHeight,false);
        double depth=positiveNumber("Extrusion depth (mm): ");
        std::filesystem::path output="sketch_run_"+std::to_string(GetTickCount64());
        if(!std::filesystem::create_directory(output))throw std::runtime_error("Cannot create a fresh output folder.");
        json reviewed=serializeSketch(drawing,imagePath);
        if(source.contains("notes"))reviewed["notes"]=source["notes"];
        if(source.contains("material"))reviewed["material"]=source["material"];
        reviewed["dimensions"]=dimensionSystem.dimensions;reviewed["constraints"]=dimensionSystem.relations;
        writeJson(output/"reviewed_sketch.json",reviewed);
        json metric=json::array();
        for(size_t i=0;i<drawing.points.size();++i){auto p=sketch::toMeters(drawing.points[i],origin,scale);
            metric.push_back({{"id",drawing.ids[i]},{"x",p.x},{"y",p.y},{"z",0}});}
        writeJson(output/"session.json",{{"image",std::filesystem::absolute(imagePath).string()},
            {"sketch_source",std::filesystem::absolute(sketchFile).string()},
            {"origin_pixels",{{"x",origin.x},{"y",origin.y}}},
            {"scale_source",dims.empty()?"two_point_calibration":"first_named_dimension"},
            {"dimension_values_mm",values},
            {"known_distance_mm",knownMm},{"mm_per_pixel",scale},{"depth_mm",depth},{"plane",plane},
            {"points_meters",metric},{"contours",reviewed["contours"]},{"preview_only",previewOnly}});
        std::cout << "Saved review: " << std::filesystem::absolute(output).string() << '\n';
        if(!previewOnly)createSolidWorksPart(drawing,origin,scale,depth,plane);
        else std::cout << "Preview only: no SOLIDWORKS part created.\n";
        return 0;
    }catch(const std::exception& e){std::cerr << "Error: " << e.what() << '\n';return 1;}
}
