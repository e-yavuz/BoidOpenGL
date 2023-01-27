#include "glad/glad.h"
#include "GLFW/glfw3.h"
#include "QuadTree/Boid.h"
#include <cstdlib>
#include <exception>
#include <iostream>
#include <fstream>
#include <random>
#include <stdexcept>
#include <vector>
#include <math.h>
#include <chrono>


void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void processInput(GLFWwindow* window);
unsigned int init_GL_Shader(std::string filePath, GLenum shaderType);
unsigned int init_GL_Program(std::vector<unsigned int> shaders);
void updateBuffer(uint &id, uint offset, void *data, uint size, GLenum shaderType);
void updateVertices(std::vector<Boid>& flock, GLFWwindow* window, std::vector<float>& vertices);
class GLFW_Wrapper
{
public:
    GLFW_Wrapper() {};
    GLFW_Wrapper(int major, int minor, int width, int height, const char* title)
    {
        glfwInit();
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, major);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, minor);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

        #ifdef __APPLE__
            glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
        #endif

        window = glfwCreateWindow(width, height, title, NULL, NULL);
        
        if (window == NULL)
        {
            glfwTerminate();
            throw std::runtime_error("Failed to create GLFW window");
        }

        glfwMakeContextCurrent(window);
        glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

        if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
            throw std::runtime_error("Failed to initialize GLAD");
    }

    static void framebuffer_size_callback(GLFWwindow* window, int width, int height)
    {
        glViewport(0, 0, width, height);
    }

    GLFWwindow* window;
};

int main()
{
    GLFW_Wrapper glfw;

    try {
        glfw = GLFW_Wrapper(3, 3, 900, 900, "Boids");
    } catch (std::runtime_error e) {
        std::cerr << e.what() << std::endl;
        return -1;
    }

    GLFWwindow* window = glfw.window;

    Flock flock;

    //DECLARE DRAW SIZE AND TOTAL NUMBER OF BOIDS
    size_t number_of_boids = (size_t)pow(2,16);
    //Found that 2^15 boids is max can be drawn at once?
    size_t draw_size = 3*(number_of_boids);

    std::srand(100);
    for(size_t i = 0; i < number_of_boids; i++)
        flock.boids.emplace_back(std::vector<float>{(static_cast<float>(std::rand())/(static_cast<float>(RAND_MAX)/2))-1, (static_cast<float>(std::rand())/(static_cast<float>(RAND_MAX)/2))-1},
                                 std::vector<float>{(static_cast<float>(std::rand())/(static_cast<float>(RAND_MAX)/2))-1, (static_cast<float>(std::rand())/(static_cast<float>(RAND_MAX)/2))-1});

    unsigned int vertexShader, fragmentShader, shaderProgram;

    try {
        vertexShader = init_GL_Shader("GLSL/V1.glsl", GL_VERTEX_SHADER);
        fragmentShader = init_GL_Shader("GLSL/F1.glsl", GL_FRAGMENT_SHADER);
        shaderProgram = init_GL_Program(std::vector<unsigned int>{vertexShader, fragmentShader});
    } catch (std::runtime_error e) {
        std::cerr << e.what() << std::endl;
        return -1;
    }



    std::vector<float> vertices(flock.boids.size()*9, 0.f);

    unsigned int VAO, VBO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices[0])*static_cast<uint>(vertices.size()), vertices.data(), GL_DYNAMIC_DRAW);
    
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3*sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glBindVertexArray(0);

    float FPS_sum = 0;
    size_t frames = 0;
    float CPS_sum = 0;
    std::chrono::high_resolution_clock::time_point start;
    // render loop
    while(!glfwWindowShouldClose(window))
    {
        processInput(window);

        //Begin CPS timer
        start = std::chrono::high_resolution_clock::now();
        //Computation Step
        std::vector<point_bucket<Boid> > tree;
        point_bucket<Boid> base(0, 0, 2, 2, flock.boids);
        base_split(base, 16, tree, 20);
        for(auto& elm: tree)
        {
            flock.Update(elm.bucket);
        }
        flock.Mirror();
        CPS_sum+=1000.f/std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()-start).count();

        // rendering commands here
        glClearColor(0.2f, 0.3f, 0.2f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        glUseProgram(shaderProgram);

        //Begin FPS timer
        start = std::chrono::high_resolution_clock::now();

        glBindVertexArray(VAO);
        updateVertices(flock.boids, window, vertices);
        updateBuffer(VBO, 0, vertices.data(), sizeof(vertices[0])*static_cast<uint>(vertices.size()), GL_ARRAY_BUFFER);
        size_t draw_running_total = vertices.size()/3;
        GLint draw_offset = 0;
        while(draw_running_total > 0)
        {
            glDrawArrays(GL_TRIANGLES, draw_offset, draw_running_total > draw_size? draw_size: draw_running_total);
            draw_running_total -= draw_size;
            draw_offset += draw_size;
        }

        // check and call events and swap the buffers
        glfwSwapBuffers(window);
        glfwPollEvents();

        // PULL FPS FOR DRAW
        FPS_sum+=1000.f/std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()-start).count();
        
        frames++;
        if(frames==100)
        {
            std::cout << "Average across 100 frames... \n" << FPS_sum/frames << " FPS\n"
            << CPS_sum/frames << " CPS\n";
            FPS_sum = 0;
            CPS_sum = 0;
            frames = 0;
        }

        //END PULL FOR FPS

    }

    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteProgram(shaderProgram);

    glfwTerminate();
    return 0;
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    glViewport(0, 0, width, height);
}

void processInput(GLFWwindow* window)
{
    if(glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
}

unsigned int init_GL_Shader(std::string filePath, GLenum shaderType)
{
    std::ifstream in(filePath);

    std::string content(std::string((std::istreambuf_iterator<char>(in)), 
    std::istreambuf_iterator<char>()));
    const char* src = content.data();

    unsigned int shader = glCreateShader(shaderType);
    glShaderSource(shader, 1, &src, NULL);
    glCompileShader(shader);

    int success;
    char info_log[512];

    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);

    if(!success)
    {
        glGetShaderInfoLog(shader, 512, NULL, info_log);
        throw std::runtime_error(info_log);
    }

    return shader;
}

unsigned int init_GL_Program(std::vector<unsigned int> shaders)
{

    unsigned int shaderProgram = glCreateProgram();
    
    for(unsigned int elm: shaders)
        glAttachShader(shaderProgram, elm);
        
    glLinkProgram(shaderProgram);

    int success;
    char info_log[512];

    glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
    if(!success) {
        glGetProgramInfoLog(shaderProgram, 512, NULL, info_log);
        throw std::runtime_error(info_log);
    }

    for(unsigned int elm: shaders)
        glDeleteShader(elm);
        
    return shaderProgram;
}

void updateBuffer(uint &id, uint offset, void *data, uint size, GLenum shaderType) 
 {
    glBindBuffer(shaderType, id);
    glBufferSubData(shaderType, offset, size, data);
}

void updateVertices(std::vector<Boid>& data, GLFWwindow* window, std::vector<float>& vertices)
{
    int width, height;
    glfwGetWindowSize(window, &width, &height);
    for(size_t ind = 0; ind < data.size(); ind++)
    {
        size_t i = ind*9;
        Boid& boid = data[ind];
        std::vector<float> directionalVector = NormalizeVectorCopy(boid.velocity, 1.f);
        directionalVector[0]/=width;
        directionalVector[1]/=height;

        vertices[i+0] = (boid.location[0]-directionalVector[1]);
        vertices[i+1] = (boid.location[1]+directionalVector[0]);

        vertices[i+3] = (boid.location[0]+directionalVector[1]);
        vertices[i+4] = (boid.location[1]-directionalVector[0]);

        for(auto& elm: directionalVector)
            elm*=4;
        vertices[i+6] = (boid.location[0]+directionalVector[0]);
        vertices[i+7] = (boid.location[1]+directionalVector[1]);
    }
}