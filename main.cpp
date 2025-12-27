#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include "shader_s.h"
#include "camera.h"
#include <iostream>
#include <vector>
#include <string>
#include <btBulletDynamicsCommon.h>

using u32 = unsigned int;

unsigned int SCR_W = 1280;
unsigned int SCR_H = 720;
const unsigned int SHADOW_WIDTH = 2048;
const unsigned int SHADOW_HEIGHT = 2048;

Camera camera(glm::vec3(0.0f, 1.6f, 3.0f));
float lastX = SCR_W * 0.5f;
float lastY = SCR_H * 0.5f;
bool firstMouse = true;
float deltaTime = 0.0f;
float lastFrame = 0.0f;
bool cameraMode = true;
bool showUI = true;

btDiscreteDynamicsWorld* dynamicsWorld = nullptr;
btBroadphaseInterface* broadphase = nullptr;
btCollisionDispatcher* dispatcher = nullptr;
btSequentialImpulseConstraintSolver* solver = nullptr;
btDefaultCollisionConfiguration* collisionConfiguration = nullptr;

btRigidBody* playerBody = nullptr;
btRigidBody* floorBody = nullptr;
std::vector<btRigidBody*> boxes;
std::vector<btCollisionShape*> collisionShapes;
std::vector<btRigidBody*> rigidBodies;

u32 cubeVAO = 0, cubeVBO = 0;
u32 floorVAO = 0, floorVBO = 0;
u32 skyboxVAO = 0, skyboxVBO = 0;
u32 jellyVAO = 0, jellyVBO = 0;
unsigned int skyboxTexture = 0;
unsigned int depthMapFBO = 0;
unsigned int depthMap = 0;

Shader* lightingShader = nullptr;
Shader* simpleShader = nullptr;
Shader* skyboxShader = nullptr;
Shader* depthShader = nullptr;

float playerHeight = 2.0f;
float playerEyeHeight = 1.8f;
float playerRadius = 0.3f;
float moveSpeed = 5.0f;
float jumpPower = 5.0f;

glm::vec3 lightPos(5.0f, 8.0f, 5.0f);
glm::vec3 lightColor(1.0f, 0.9f, 0.8f);
float lightNearPlane = 1.0f;
float lightFarPlane = 30.0f;
glm::mat4 lightSpaceMatrix;

float physicsTimeAccumulator = 0.0f;
const float physicsTimeStep = 1.0f / 60.0f;

bool showWireframe = false;
bool shadowsEnabled = true;

float bulletSize = 0.3f;
float bulletForce = 30.0f;
float bulletMass = 0.5f;
bool ePressed = false;
bool enableJellyCollision = true;

static glm::vec3 btToGlm(const btVector3& v) {
    return glm::vec3(v.x(), v.y(), v.z());
}

static btVector3 glmToBt(const glm::vec3& v) {
    return btVector3(v.x, v.y, v.z);
}

void setup_catppuccin_mocha_theme() {
    ImGuiStyle& style = ImGui::GetStyle();
    ImVec4* colors = style.Colors;

    const ImVec4 base = ImVec4(0.117f, 0.117f, 0.172f, 1.0f);
    const ImVec4 mantle = ImVec4(0.109f, 0.109f, 0.156f, 1.0f);
    const ImVec4 surface0 = ImVec4(0.200f, 0.207f, 0.286f, 1.0f);
    const ImVec4 surface1 = ImVec4(0.247f, 0.254f, 0.337f, 1.0f);
    const ImVec4 surface2 = ImVec4(0.290f, 0.301f, 0.388f, 1.0f);
    const ImVec4 overlay0 = ImVec4(0.396f, 0.403f, 0.486f, 1.0f);
    const ImVec4 overlay2 = ImVec4(0.576f, 0.584f, 0.654f, 1.0f);
    const ImVec4 text = ImVec4(0.803f, 0.815f, 0.878f, 1.0f);
    const ImVec4 subtext0 = ImVec4(0.639f, 0.658f, 0.764f, 1.0f);
    const ImVec4 mauve = ImVec4(0.796f, 0.698f, 0.972f, 1.0f);
    const ImVec4 peach = ImVec4(0.980f, 0.709f, 0.572f, 1.0f);
    const ImVec4 yellow = ImVec4(0.980f, 0.913f, 0.596f, 1.0f);
    const ImVec4 green = ImVec4(0.650f, 0.890f, 0.631f, 1.0f);
    const ImVec4 teal = ImVec4(0.580f, 0.886f, 0.819f, 1.0f);
    const ImVec4 sapphire = ImVec4(0.458f, 0.784f, 0.878f, 1.0f);
    const ImVec4 blue = ImVec4(0.533f, 0.698f, 0.976f, 1.0f);
    const ImVec4 lavender = ImVec4(0.709f, 0.764f, 0.980f, 1.0f);

    colors[ImGuiCol_WindowBg] = base;
    colors[ImGuiCol_ChildBg] = base;
    colors[ImGuiCol_PopupBg] = surface0;
    colors[ImGuiCol_Border] = surface1;
    colors[ImGuiCol_BorderShadow] = ImVec4(0.0f, 0.0f, 0.0f, 0.0f);
    colors[ImGuiCol_FrameBg] = surface0;
    colors[ImGuiCol_FrameBgHovered] = surface1;
    colors[ImGuiCol_FrameBgActive] = surface2;
    colors[ImGuiCol_TitleBg] = mantle;
    colors[ImGuiCol_TitleBgActive] = surface0;
    colors[ImGuiCol_TitleBgCollapsed] = mantle;
    colors[ImGuiCol_MenuBarBg] = mantle;
    colors[ImGuiCol_ScrollbarBg] = surface0;
    colors[ImGuiCol_ScrollbarGrab] = surface2;
    colors[ImGuiCol_ScrollbarGrabHovered] = overlay0;
    colors[ImGuiCol_ScrollbarGrabActive] = overlay2;
    colors[ImGuiCol_CheckMark] = green;
    colors[ImGuiCol_SliderGrab] = sapphire;
    colors[ImGuiCol_SliderGrabActive] = blue;
    colors[ImGuiCol_Button] = surface0;
    colors[ImGuiCol_ButtonHovered] = surface1;
    colors[ImGuiCol_ButtonActive] = surface2;
    colors[ImGuiCol_Header] = surface0;
    colors[ImGuiCol_HeaderHovered] = surface1;
    colors[ImGuiCol_HeaderActive] = surface2;
    colors[ImGuiCol_Separator] = surface1;
    colors[ImGuiCol_SeparatorHovered] = mauve;
    colors[ImGuiCol_SeparatorActive] = mauve;
    colors[ImGuiCol_ResizeGrip] = surface2;
    colors[ImGuiCol_ResizeGripHovered] = mauve;
    colors[ImGuiCol_ResizeGripActive] = mauve;
    colors[ImGuiCol_Tab] = surface0;
    colors[ImGuiCol_TabHovered] = surface2;
    colors[ImGuiCol_TabActive] = surface1;
    colors[ImGuiCol_TabUnfocused] = surface0;
    colors[ImGuiCol_TabUnfocusedActive] = surface1;
    colors[ImGuiCol_PlotLines] = blue;
    colors[ImGuiCol_PlotLinesHovered] = peach;
    colors[ImGuiCol_PlotHistogram] = teal;
    colors[ImGuiCol_PlotHistogramHovered] = green;
    colors[ImGuiCol_TableHeaderBg] = surface0;
    colors[ImGuiCol_TableBorderStrong] = surface1;
    colors[ImGuiCol_TableBorderLight] = surface0;
    colors[ImGuiCol_TableRowBg] = ImVec4(0.0f, 0.0f, 0.0f, 0.0f);
    colors[ImGuiCol_TableRowBgAlt] = ImVec4(1.0f, 1.0f, 1.0f, 0.06f);
    colors[ImGuiCol_TextSelectedBg] = surface2;
    colors[ImGuiCol_DragDropTarget] = yellow;
    colors[ImGuiCol_NavHighlight] = lavender;
    colors[ImGuiCol_NavWindowingHighlight] = ImVec4(1.0f, 1.0f, 1.0f, 0.7f);
    colors[ImGuiCol_NavWindowingDimBg] = ImVec4(0.8f, 0.8f, 0.8f, 0.2f);
    colors[ImGuiCol_ModalWindowDimBg] = ImVec4(0.0f, 0.0f, 0.0f, 0.35f);
    colors[ImGuiCol_Text] = text;
    colors[ImGuiCol_TextDisabled] = subtext0;

    style.WindowRounding = 6.0f;
    style.ChildRounding = 6.0f;
    style.FrameRounding = 4.0f;
    style.PopupRounding = 4.0f;
    style.ScrollbarRounding = 9.0f;
    style.GrabRounding = 4.0f;
    style.TabRounding = 4.0f;

    style.WindowPadding = ImVec2(8.0f, 8.0f);
    style.FramePadding = ImVec2(5.0f, 3.0f);
    style.ItemSpacing = ImVec2(8.0f, 4.0f);
    style.ItemInnerSpacing = ImVec2(4.0f, 4.0f);
    style.IndentSpacing = 21.0f;
    style.ScrollbarSize = 14.0f;
    style.GrabMinSize = 10.0f;

    style.WindowBorderSize = 1.0f;
    style.ChildBorderSize = 1.0f;
    style.PopupBorderSize = 1.0f;
    style.FrameBorderSize = 0.0f;
    style.TabBorderSize = 0.0f;
}

btRigidBody* addStaticBox(const btVector3& halfExtents, const btVector3& position) {
    btCollisionShape* shape = new btBoxShape(halfExtents);
    collisionShapes.push_back(shape);
    btTransform t;
    t.setIdentity();
    t.setOrigin(position);
    btScalar mass = 0.0f;
    btVector3 localInertia(0, 0, 0);
    btDefaultMotionState* motion = new btDefaultMotionState(t);
    btRigidBody::btRigidBodyConstructionInfo info(mass, motion, shape, localInertia);
    btRigidBody* body = new btRigidBody(info);
    dynamicsWorld->addRigidBody(body);
    rigidBodies.push_back(body);
    return body;
}

btRigidBody* addDynamicBox(const btVector3& halfExtents, const btVector3& position, btScalar mass) {
    btCollisionShape* shape = new btBoxShape(halfExtents);
    collisionShapes.push_back(shape);
    btTransform t;
    t.setIdentity();
    t.setOrigin(position);
    btVector3 localInertia(0, 0, 0);
    shape->calculateLocalInertia(mass, localInertia);
    btDefaultMotionState* motion = new btDefaultMotionState(t);
    btRigidBody::btRigidBodyConstructionInfo info(mass, motion, shape, localInertia);
    btRigidBody* body = new btRigidBody(info);
    body->setFriction(0.5f);
    body->setRestitution(0.2f);
    dynamicsWorld->addRigidBody(body);
    rigidBodies.push_back(body);
    boxes.push_back(body);
    return body;
}

struct Spring {
    int pointA;
    int pointB;
    float restLength;
};

struct MassPoint {
    glm::vec3 position;
    glm::vec3 oldPosition;
    glm::vec3 acceleration;
    float mass;
    bool fixed;
    glm::vec3 normal;
    glm::vec3 color;
};

class JellyCube {
private:
    unsigned int jellyEBO = 0;
    bool buffersInitialized = false;
public:
    std::vector<MassPoint> points;
    std::vector<Spring> springs;
    std::vector<unsigned int> indices;
    glm::vec3 center;
    float size;
    float stiffness = 500.0f;
    float damping = 0.98f;

    JellyCube(glm::vec3 pos = glm::vec3(0.0f, 3.0f, 0.0f), float s = 1.5f)
        : center(pos), size(s) {
        InitializeCube();
    }

    ~JellyCube() {
        if (jellyEBO) {
            glDeleteBuffers(1, &jellyEBO);
            jellyEBO = 0;
        }
    }

    void InitializeBuffers() {
        if (!buffersInitialized) {
            if (!jellyEBO) {
                glGenBuffers(1, &jellyEBO);
            }
            buffersInitialized = true;
        }
    }

    void InitializeCube() {
        points.clear();
        springs.clear();
        indices.clear();
        glm::vec3 corners[] = {
            glm::vec3(-0.5f, -0.5f, -0.5f),
            glm::vec3(0.5f, -0.5f, -0.5f),
            glm::vec3(0.5f, 0.5f, -0.5f),
            glm::vec3(-0.5f, 0.5f, -0.5f),
            glm::vec3(-0.5f, -0.5f, 0.5f),
            glm::vec3(0.5f, -0.5f, 0.5f),
            glm::vec3(0.5f, 0.5f, 0.5f),
            glm::vec3(-0.5f, 0.5f, 0.5f)
        };
        glm::vec3 colors[] = {
            glm::vec3(0.2f, 0.8f, 0.2f),
            glm::vec3(0.2f, 0.8f, 0.2f),
            glm::vec3(0.2f, 0.8f, 0.2f),
            glm::vec3(0.2f, 0.8f, 0.2f),
            glm::vec3(0.2f, 0.8f, 0.2f),
            glm::vec3(0.2f, 0.8f, 0.2f),
            glm::vec3(0.2f, 0.8f, 0.2f),
            glm::vec3(0.2f, 0.8f, 0.2f)
        };
        for (int i = 0; i < 8; i++) {
            MassPoint p;
            p.position = center + corners[i] * size;
            p.oldPosition = p.position;
            p.acceleration = glm::vec3(0.0f);
            p.mass = 1.0f;
            p.fixed = false;
            p.color = colors[i];
            p.normal = glm::vec3(0.0f);
            points.push_back(p);
        }
        unsigned int cubeIndices[] = {
            0, 1, 2, 2, 3, 0,
            4, 6, 5, 6, 4, 7,
            0, 3, 7, 7, 4, 0,
            1, 5, 6, 6, 2, 1,
            0, 4, 5, 5, 1, 0,
            3, 2, 6, 6, 7, 3
        };
        indices.assign(cubeIndices, cubeIndices + 36);
        AddSpring(0, 1, size);
        AddSpring(1, 2, size);
        AddSpring(2, 3, size);
        AddSpring(3, 0, size);
        AddSpring(4, 5, size);
        AddSpring(5, 6, size);
        AddSpring(6, 7, size);
        AddSpring(7, 4, size);
        AddSpring(0, 4, size);
        AddSpring(1, 5, size);
        AddSpring(2, 6, size);
        AddSpring(3, 7, size);
        AddSpring(0, 2, size * 1.414f);
        AddSpring(1, 3, size * 1.414f);
        AddSpring(4, 6, size * 1.414f);
        AddSpring(5, 7, size * 1.414f);
        AddSpring(0, 7, size * 1.414f);
        AddSpring(3, 4, size * 1.414f);
        AddSpring(1, 6, size * 1.414f);
        AddSpring(2, 5, size * 1.414f);
        AddSpring(0, 6, size * 1.732f);
        AddSpring(1, 7, size * 1.732f);
        AddSpring(2, 4, size * 1.732f);
        AddSpring(3, 5, size * 1.732f);
    }

    void AddSpring(int a, int b, float restLength) {
        Spring s;
        s.pointA = a;
        s.pointB = b;
        s.restLength = restLength;
        springs.push_back(s);
    }

    void Update(float dt) {
        if (dt <= 0.0f) return;
        for (auto& point : points) {
            if (point.fixed) continue;
            point.acceleration = glm::vec3(0.0f, -9.8f, 0.0f);
            glm::vec3 temp = point.position;
            point.position = point.position + (point.position - point.oldPosition) * damping
                + point.acceleration * dt * dt;
            point.oldPosition = temp;
        }
        for (int i = 0; i < 10; i++) {
            for (const auto& spring : springs) {
                if (spring.pointA >= points.size() || spring.pointB >= points.size()) continue;
                MassPoint& a = points[spring.pointA];
                MassPoint& b = points[spring.pointB];
                if (a.fixed && b.fixed) continue;
                glm::vec3 delta = b.position - a.position;
                float distance = glm::length(delta);
                if (distance > 0.0f) {
                    float displacement = distance - spring.restLength;
                    glm::vec3 forceDir = glm::normalize(delta);
                    glm::vec3 springForce = stiffness * displacement * forceDir;
                    if (!a.fixed) {
                        a.position += springForce * dt * dt * 0.5f / a.mass;
                    }
                    if (!b.fixed) {
                        b.position -= springForce * dt * dt * 0.5f / b.mass;
                    }
                }
            }
        }
        for (auto& point : points) {
            if (point.position.y < 0.0f) {
                point.position.y = 0.0f;
                glm::vec3 velocity = (point.position - point.oldPosition) / dt;
                velocity.y = -velocity.y * 0.3f;
                point.oldPosition = point.position - velocity * dt * damping;
            }
        }
        if (enableJellyCollision) {
            for (auto& point : points) {
                for (btRigidBody* box : boxes) {
                    if (!box || box == playerBody || box == floorBody) continue;
                    btTransform transform;
                    box->getMotionState()->getWorldTransform(transform);
                    btVector3 boxPos = transform.getOrigin();
                    float collisionRadius = 1.0f;
                    glm::vec3 toPoint = point.position - btToGlm(boxPos);
                    float distance = glm::length(toPoint);
                    if (distance < collisionRadius && distance > 0.0f) {
                        glm::vec3 pushDir = glm::normalize(toPoint);
                        point.position = btToGlm(boxPos) + pushDir * collisionRadius;
                        btVector3 boxVel = box->getLinearVelocity();
                        box->setLinearVelocity(boxVel + glmToBt(-pushDir * 2.0f));
                    }
                }
            }
        }
        for (auto& point : points) {
            point.normal = glm::vec3(0.0f);
        }
        for (size_t i = 0; i < indices.size(); i += 3) {
            MassPoint& a = points[indices[i]];
            MassPoint& b = points[indices[i + 1]];
            MassPoint& c = points[indices[i + 2]];
            glm::vec3 edge1 = b.position - a.position;
            glm::vec3 edge2 = c.position - a.position;
            glm::vec3 normal = glm::normalize(glm::cross(edge1, edge2));
            a.normal += normal;
            b.normal += normal;
            c.normal += normal;
        }
        for (auto& point : points) {
            point.normal = glm::normalize(point.normal);
        }
    }

    void UpdateBuffers() {
        if (points.empty() || !buffersInitialized) return;
        std::vector<float> vertices;
        vertices.reserve(points.size() * 9);
        for (const auto& point : points) {
            vertices.push_back(point.position.x);
            vertices.push_back(point.position.y);
            vertices.push_back(point.position.z);
            vertices.push_back(point.normal.x);
            vertices.push_back(point.normal.y);
            vertices.push_back(point.normal.z);
            vertices.push_back(point.color.r);
            vertices.push_back(point.color.g);
            vertices.push_back(point.color.b);
        }
        glBindVertexArray(jellyVAO);
        glBindBuffer(GL_ARRAY_BUFFER, jellyVBO);
        glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float),
            vertices.data(), GL_DYNAMIC_DRAW);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, jellyEBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int),
            indices.data(), GL_DYNAMIC_DRAW);
        glBindVertexArray(0);
    }

    void ApplyImpulse(glm::vec3 force) {
        for (auto& point : points) {
            if (!point.fixed) {
                glm::vec3 velocity = (point.position - point.oldPosition) / deltaTime;
                velocity += force / point.mass;
                point.oldPosition = point.position - velocity * deltaTime;
            }
        }
    }

    void Draw() {
        if (!buffersInitialized || points.empty() || indices.empty()) return;
        UpdateBuffers();
        glBindVertexArray(jellyVAO);
        glDrawElements(GL_TRIANGLES, (GLsizei)indices.size(), GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);
    }

    void DrawWireframe() {
        if (!buffersInitialized || points.empty() || indices.empty()) return;
        UpdateBuffers();
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glBindVertexArray(jellyVAO);
        glDrawElements(GL_TRIANGLES, (GLsizei)indices.size(), GL_UNSIGNED_INT, 0);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glBindVertexArray(0);
    }
};

JellyCube* jellyCube = nullptr;
float jellyStiffness = 500.0f;
float jellyDamping = 0.98f;
float jellyImpulse = 15.0f;

unsigned int loadSkybox() {
    std::vector<std::string> faces = {
        "textures/skybox/right.png",
        "textures/skybox/left.png",
        "textures/skybox/top.png",
        "textures/skybox/bottom.png",
        "textures/skybox/front.png",
        "textures/skybox/back.png"
    };
    unsigned int textureID;
    glGenTextures(1, &textureID);
    glBindTexture(GL_TEXTURE_CUBE_MAP, textureID);
    int width, height, nrChannels;
    for (unsigned int i = 0; i < faces.size(); i++) {
        unsigned char* data = stbi_load(faces[i].c_str(), &width, &height, &nrChannels, 0);
        if (!data) {
            std::cout << "Failed to load skybox texture: " << faces[i] << std::endl;
            unsigned char blue[] = { 100, 150, 255 };
            for (unsigned int j = 0; j < 6; j++) {
                glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + j, 0, GL_RGB,
                    1, 1, 0, GL_RGB, GL_UNSIGNED_BYTE, blue);
            }
            stbi_image_free(data);
            break;
        }
        GLenum format = GL_RGB;
        if (nrChannels == 1) format = GL_RED;
        else if (nrChannels == 3) format = GL_RGB;
        else if (nrChannels == 4) format = GL_RGBA;
        glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, format,
            width, height, 0, format, GL_UNSIGNED_BYTE, data);
        stbi_image_free(data);
    }
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
    return textureID;
}

void configureDepthMap() {
    glGenFramebuffers(1, &depthMapFBO);
    glGenTextures(1, &depthMap);
    glBindTexture(GL_TEXTURE_2D, depthMap);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT,
        SHADOW_WIDTH, SHADOW_HEIGHT, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    float borderColor[] = { 1.0f, 1.0f, 1.0f, 1.0f };
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, borderColor);
    glBindFramebuffer(GL_FRAMEBUFFER, depthMapFBO);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depthMap, 0);
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        std::cerr << "ERROR::FRAMEBUFFER:: Depth framebuffer is not complete!" << std::endl;
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void cleanup() {
    if (dynamicsWorld) {
        for (int i = dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--) {
            btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[i];
            btRigidBody* body = btRigidBody::upcast(obj);
            if (body && body->getMotionState()) {
                delete body->getMotionState();
            }
            if (obj) {
                dynamicsWorld->removeCollisionObject(obj);
            }
        }
    }

    for (auto shape : collisionShapes) {
        if (shape) delete shape;
    }
    collisionShapes.clear();

    for (auto body : rigidBodies) {
        if (body) {
            if (body->getMotionState()) {
                delete body->getMotionState();
            }
            delete body;
        }
    }
    rigidBodies.clear();

    boxes.clear();

    if (jellyCube) {
        delete jellyCube;
        jellyCube = nullptr;
    }

    if (dynamicsWorld) delete dynamicsWorld;
    if (solver) delete solver;
    if (broadphase) delete broadphase;
    if (dispatcher) delete dispatcher;
    if (collisionConfiguration) delete collisionConfiguration;

    if (cubeVAO) glDeleteVertexArrays(1, &cubeVAO);
    if (cubeVBO) glDeleteBuffers(1, &cubeVBO);
    if (floorVAO) glDeleteVertexArrays(1, &floorVAO);
    if (floorVBO) glDeleteBuffers(1, &floorVBO);
    if (skyboxVAO) glDeleteVertexArrays(1, &skyboxVAO);
    if (skyboxVBO) glDeleteBuffers(1, &skyboxVBO);
    if (jellyVAO) glDeleteVertexArrays(1, &jellyVAO);
    if (jellyVBO) glDeleteBuffers(1, &jellyVBO);
    if (skyboxTexture) glDeleteTextures(1, &skyboxTexture);
    if (depthMapFBO) glDeleteFramebuffers(1, &depthMapFBO);
    if (depthMap) glDeleteTextures(1, &depthMap);

    if (lightingShader) delete lightingShader;
    if (simpleShader) delete simpleShader;
    if (skyboxShader) delete skyboxShader;
    if (depthShader) delete depthShader;
}

int main() {
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return -1;
    }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    GLFWwindow* window = glfwCreateWindow(SCR_W, SCR_H, "Flux3D Engine", NULL, NULL);
    if (!window) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, [](GLFWwindow* window, int width, int height) {
        if (width <= 0 || height <= 0) return;
        SCR_W = width;
        SCR_H = height;
        glViewport(0, 0, width, height);
        });
    glfwSetCursorPosCallback(window, [](GLFWwindow* window, double xpos, double ypos) {
        if (!cameraMode) return;
        if (firstMouse) {
            lastX = (float)xpos;
            lastY = (float)ypos;
            firstMouse = false;
        }
        float xoffset = (float)xpos - lastX;
        float yoffset = lastY - (float)ypos;
        lastX = (float)xpos;
        lastY = (float)ypos;
        camera.ProcessMouseMovement(xoffset, yoffset);
        });
    glfwSetScrollCallback(window, [](GLFWwindow* window, double xoffset, double yoffset) {
        camera.ProcessMouseScroll((float)yoffset);
        });
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "Failed to initialize GLAD" << std::endl;
        glfwTerminate();
        return -1;
    }
    std::cout << "OpenGL Version: " << glGetString(GL_VERSION) << std::endl;
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Инициализация Bullet Physics
    collisionConfiguration = new btDefaultCollisionConfiguration();
    dispatcher = new btCollisionDispatcher(collisionConfiguration);
    broadphase = new btDbvtBroadphase();
    solver = new btSequentialImpulseConstraintSolver();
    dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
    dynamicsWorld->setGravity(btVector3(0.0f, -9.81f, 0.0f));

    // Пол
    floorBody = addStaticBox(btVector3(50.0f, 0.5f, 50.0f), btVector3(0.0f, -0.5f, 0.0f));

    // Кубы НАДО КАСАТЬСЯ пола, а не быть над ним
    addDynamicBox(btVector3(1.0f, 1.0f, 1.0f), btVector3(-4.0f, 1.0f, 0.0f), 5.0f);  // y=1.0 означает что центр куба на высоте 1м, а его низ на высоте 0
    addDynamicBox(btVector3(0.8f, 0.8f, 0.8f), btVector3(4.0f, 0.8f, 0.0f), 3.0f);  // y=0.8 означает что центр куба на высоте 0.8м

    // Игрок
    btCollisionShape* shape = new btCapsuleShape(playerRadius, playerHeight - 2.0f * playerRadius);
    collisionShapes.push_back(shape);
    btTransform t;
    t.setIdentity();
    t.setOrigin(btVector3(0.0f, 1.6f, 0.0f));
    btScalar mass = 80.0f;
    btVector3 inertia(0, 0, 0);
    shape->calculateLocalInertia(mass, inertia);
    btDefaultMotionState* motion = new btDefaultMotionState(t);
    btRigidBody::btRigidBodyConstructionInfo info(mass, motion, shape, inertia);
    playerBody = new btRigidBody(info);
    playerBody->setAngularFactor(btVector3(0, 0, 0));
    playerBody->setFriction(0.8f);
    playerBody->setRestitution(0.0f);
    playerBody->setSleepingThresholds(0.0f, 0.0f);
    dynamicsWorld->addRigidBody(playerBody);
    rigidBodies.push_back(playerBody);

    // Шейдеры
    lightingShader = new Shader("shaders/lighting.vert", "shaders/lighting.frag");
    simpleShader = new Shader("shaders/simple.vert", "shaders/simple.frag");
    skyboxShader = new Shader("shaders/skybox.vert", "shaders/skybox.frag");
    depthShader = new Shader("shaders/depth.vert", "shaders/depth.frag");

    float cubeVerts[] = {
        -0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
         0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
         0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
         0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
        -0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
        -0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
        -0.5f, -0.5f,  0.5f,  0.0f,  0.0f, 1.0f,
         0.5f, -0.5f,  0.5f,  0.0f,  0.0f, 1.0f,
         0.5f,  0.5f,  0.5f,  0.0f,  0.0f, 1.0f,
         0.5f,  0.5f,  0.5f,  0.0f,  0.0f, 1.0f,
        -0.5f,  0.5f,  0.5f,  0.0f,  0.0f, 1.0f,
        -0.5f, -0.5f,  0.5f,  0.0f,  0.0f, 1.0f,
        -0.5f,  0.5f,  0.5f, -1.0f,  0.0f,  0.0f,
        -0.5f,  0.5f, -0.5f, -1.0f,  0.0f,  0.0f,
        -0.5f, -0.5f, -0.5f, -1.0f,  0.0f,  0.0f,
        -0.5f, -0.5f, -0.5f, -1.0f,  0.0f,  0.0f,
        -0.5f, -0.5f,  0.5f, -1.0f,  0.0f,  0.0f,
        -0.5f,  0.5f,  0.5f, -1.0f,  0.0f,  0.0f,
         0.5f,  0.5f,  0.5f,  1.0f,  0.0f,  0.0f,
         0.5f,  0.5f, -0.5f,  1.0f,  0.0f,  0.0f,
         0.5f, -0.5f, -0.5f,  1.0f,  0.0f,  0.0f,
         0.5f, -0.5f, -0.5f,  1.0f,  0.0f,  0.0f,
         0.5f, -0.5f,  0.5f,  1.0f,  0.0f,  0.0f,
         0.5f,  0.5f,  0.5f,  1.0f,  0.0f,  0.0f,
         -0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,
          0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,
          0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,
          0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,
         -0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,
         -0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,
         -0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f,
          0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f,
          0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,
          0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,
         -0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,
         -0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f
    };
    glGenVertexArrays(1, &cubeVAO);
    glGenBuffers(1, &cubeVBO);
    glBindVertexArray(cubeVAO);
    glBindBuffer(GL_ARRAY_BUFFER, cubeVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(cubeVerts), cubeVerts, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glBindVertexArray(0);

    float floorVerts[] = {
        -50.0f, 0.0f, -50.0f,  0.0f, 1.0f, 0.0f,
         50.0f, 0.0f, -50.0f,  0.0f, 1.0f, 0.0f,
         50.0f, 0.0f,  50.0f,  0.0f, 1.0f, 0.0f,
         50.0f, 0.0f,  50.0f,  0.0f, 1.0f, 0.0f,
        -50.0f, 0.0f,  50.0f,  0.0f, 1.0f, 0.0f,
        -50.0f, 0.0f, -50.0f,  0.0f, 1.0f, 0.0f
    };
    glGenVertexArrays(1, &floorVAO);
    glGenBuffers(1, &floorVBO);
    glBindVertexArray(floorVAO);
    glBindBuffer(GL_ARRAY_BUFFER, floorVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(floorVerts), floorVerts, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glBindVertexArray(0);

    float skyboxVertices[] = {
        -1.0f,  1.0f, -1.0f,
        -1.0f, -1.0f, -1.0f,
         1.0f, -1.0f, -1.0f,
         1.0f, -1.0f, -1.0f,
         1.0f,  1.0f, -1.0f,
        -1.0f,  1.0f, -1.0f,
        -1.0f, -1.0f,  1.0f,
        -1.0f, -1.0f, -1.0f,
        -1.0f,  1.0f, -1.0f,
        -1.0f,  1.0f, -1.0f,
        -1.0f,  1.0f,  1.0f,
        -1.0f, -1.0f,  1.0f,
         1.0f, -1.0f, -1.0f,
         1.0f, -1.0f,  1.0f,
         1.0f,  1.0f,  1.0f,
         1.0f,  1.0f,  1.0f,
         1.0f,  1.0f, -1.0f,
         1.0f, -1.0f, -1.0f,
        -1.0f, -1.0f,  1.0f,
        -1.0f,  1.0f,  1.0f,
         1.0f,  1.0f,  1.0f,
         1.0f,  1.0f,  1.0f,
         1.0f, -1.0f,  1.0f,
        -1.0f, -1.0f,  1.0f,
        -1.0f,  1.0f, -1.0f,
         1.0f,  1.0f, -1.0f,
         1.0f,  1.0f,  1.0f,
         1.0f,  1.0f,  1.0f,
        -1.0f,  1.0f,  1.0f,
        -1.0f,  1.0f, -1.0f,
        -1.0f, -1.0f, -1.0f,
        -1.0f, -1.0f,  1.0f,
         1.0f, -1.0f, -1.0f,
         1.0f, -1.0f, -1.0f,
        -1.0f, -1.0f,  1.0f,
         1.0f, -1.0f,  1.0f
    };
    glGenVertexArrays(1, &skyboxVAO);
    glGenBuffers(1, &skyboxVBO);
    glBindVertexArray(skyboxVAO);
    glBindBuffer(GL_ARRAY_BUFFER, skyboxVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(skyboxVertices), skyboxVertices, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);

    if (jellyVAO == 0) glGenVertexArrays(1, &jellyVAO);
    if (jellyVBO == 0) glGenBuffers(1, &jellyVBO);
    glBindVertexArray(jellyVAO);
    glBindBuffer(GL_ARRAY_BUFFER, jellyVBO);
    glBufferData(GL_ARRAY_BUFFER, 8 * 9 * sizeof(float), nullptr, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), (void*)(6 * sizeof(float)));
    glEnableVertexAttribArray(2);
    glBindVertexArray(0);

    jellyCube = new JellyCube(glm::vec3(0.0f, 3.0f, 0.0f), 1.5f);
    jellyCube->InitializeBuffers();
    configureDepthMap();
    skyboxTexture = loadSkybox();

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");
    setup_catppuccin_mocha_theme();

    lastFrame = (float)glfwGetTime();
    std::cout << "Starting Flux3D Engine..." << std::endl;

    // Главный цикл
    while (!glfwWindowShouldClose(window)) {
        float current = (float)glfwGetTime();
        deltaTime = current - lastFrame;
        if (deltaTime <= 0.0f || deltaTime > 0.5f) deltaTime = 1.0f / 60.0f;
        lastFrame = current;

        // Обработка ввода
        static bool tabPressed = false;
        static bool spacePressed = false;
        static bool eWasPressed = false;
        static bool f1Pressed = false;

        if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
            glfwSetWindowShouldClose(window, true);

        // F1 для переключения ImGui
        bool f1Down = (glfwGetKey(window, GLFW_KEY_F1) == GLFW_PRESS);
        if (f1Down && !f1Pressed) {
            showUI = !showUI;
            f1Pressed = true;
        }
        if (!f1Down) f1Pressed = false;

        bool tabDown = (glfwGetKey(window, GLFW_KEY_TAB) == GLFW_PRESS);
        if (tabDown && !tabPressed) {
            cameraMode = !cameraMode;
            if (cameraMode) {
                glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
                firstMouse = true;
            }
            else {
                glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
            }
            tabPressed = true;
        }
        if (!tabDown) tabPressed = false;

        // Управление игроком
        if (cameraMode && playerBody) {
            glm::vec3 moveDir(0.0f);
            if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) moveDir += camera.Front;
            if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) moveDir -= camera.Front;
            if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) moveDir -= glm::normalize(glm::cross(camera.Front, camera.Up));
            if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) moveDir += glm::normalize(glm::cross(camera.Front, camera.Up));
            moveDir.y = 0.0f;
            if (glm::length(moveDir) > 0.001f) {
                moveDir = glm::normalize(moveDir) * moveSpeed;
                btVector3 vel = playerBody->getLinearVelocity();
                playerBody->setLinearVelocity(btVector3(moveDir.x, vel.getY(), moveDir.z));
            }
            else {
                btVector3 vel = playerBody->getLinearVelocity();
                playerBody->setLinearVelocity(btVector3(vel.getX() * 0.9f, vel.getY(), vel.getZ() * 0.9f));
            }
            if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS && !spacePressed) {
                btTransform t;
                playerBody->getMotionState()->getWorldTransform(t);
                btVector3 pos = t.getOrigin();
                if (pos.getY() < 1.7f) {
                    btVector3 vel = playerBody->getLinearVelocity();
                    vel.setY(jumpPower);
                    playerBody->setLinearVelocity(vel);
                }
                spacePressed = true;
            }
            if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_RELEASE) spacePressed = false;
        }

        // Стрельба на клавишу E
        bool eKeyDown = (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS);
        if (eKeyDown && !eWasPressed) {
            glm::vec3 startPos = camera.Position + camera.Front * 1.5f;
            btRigidBody* bullet = addDynamicBox(
                btVector3(bulletSize, bulletSize, bulletSize),
                glmToBt(startPos),
                bulletMass
            );
            bullet->setRestitution(0.5f);
            bullet->setFriction(0.3f);
            glm::vec3 direction = camera.Front;
            btVector3 impulse = glmToBt(direction * bulletForce);
            bullet->applyCentralImpulse(impulse);

            eWasPressed = true;
        }
        if (!eKeyDown) eWasPressed = false;

        // Физика
        physicsTimeAccumulator += deltaTime;
        while (physicsTimeAccumulator >= physicsTimeStep) {
            dynamicsWorld->stepSimulation(physicsTimeStep, 10);
            physicsTimeAccumulator -= physicsTimeStep;
        }

        // Обновление желе
        if (jellyCube) {
            jellyCube->stiffness = jellyStiffness;
            jellyCube->damping = jellyDamping;
            jellyCube->Update(deltaTime);
        }

        // Обновление позиции камеры
        if (playerBody) {
            btTransform pt;
            playerBody->getMotionState()->getWorldTransform(pt);
            btVector3 pos = pt.getOrigin();
            camera.Position = btToGlm(pos) + glm::vec3(0.0f, playerEyeHeight - 0.2f, 0.0f);
        }

        // Рендеринг теней
        if (shadowsEnabled) {
            glm::mat4 lightProjection = glm::ortho(-20.0f, 20.0f, -20.0f, 20.0f, lightNearPlane, lightFarPlane);
            glm::mat4 lightView = glm::lookAt(lightPos, glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
            lightSpaceMatrix = lightProjection * lightView;
            depthShader->use();
            depthShader->setMat4("lightSpaceMatrix", lightSpaceMatrix);
            glViewport(0, 0, SHADOW_WIDTH, SHADOW_HEIGHT);
            glBindFramebuffer(GL_FRAMEBUFFER, depthMapFBO);
            glClear(GL_DEPTH_BUFFER_BIT);
            glCullFace(GL_FRONT);
            glm::mat4 model = glm::mat4(1.0f);
            depthShader->setMat4("model", model);
            glBindVertexArray(floorVAO);
            glDrawArrays(GL_TRIANGLES, 0, 6);
            for (btRigidBody* rb : rigidBodies) {
                if (!rb || rb == playerBody || rb == floorBody) continue;
                btTransform tr;
                rb->getMotionState()->getWorldTransform(tr);
                btVector3 origin = tr.getOrigin();
                btMatrix3x3 basis = tr.getBasis();
                model = glm::mat4(1.0f);
                model = glm::translate(model, btToGlm(origin));
                glm::mat4 rot(1.0f);
                rot[0][0] = basis[0][0]; rot[1][0] = basis[0][1]; rot[2][0] = basis[0][2];
                rot[0][1] = basis[1][0]; rot[1][1] = basis[1][1]; rot[2][1] = basis[1][2];
                rot[0][2] = basis[2][0]; rot[1][2] = basis[2][1]; rot[2][2] = basis[2][2];
                model *= rot;
                const btCollisionShape* shape = rb->getCollisionShape();
                if (shape->getShapeType() == BOX_SHAPE_PROXYTYPE) {
                    const btBoxShape* bx = static_cast<const btBoxShape*>(shape);
                    btVector3 he = bx->getHalfExtentsWithMargin();
                    model = glm::scale(model, glm::vec3(he.x(), he.y(), he.z()));
                }
                depthShader->setMat4("model", model);
                glBindVertexArray(cubeVAO);
                glDrawArrays(GL_TRIANGLES, 0, 36);
            }
            model = glm::mat4(1.0f);
            depthShader->setMat4("model", model);
            if (jellyCube) {
                if (showWireframe) {
                    jellyCube->DrawWireframe();
                }
                else {
                    jellyCube->Draw();
                }
            }
            glCullFace(GL_BACK);
            glBindFramebuffer(GL_FRAMEBUFFER, 0);
            glViewport(0, 0, SCR_W, SCR_H);
        }

        // Очистка экрана
        glClearColor(0.05f, 0.05f, 0.08f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), (float)SCR_W / (float)SCR_H, 0.1f, 200.0f);
        glm::mat4 view = camera.GetViewMatrix();

        // Рендеринг скайбокса
        glDepthFunc(GL_LEQUAL);
        skyboxShader->use();
        skyboxShader->setMat4("projection", projection);
        skyboxShader->setMat4("view", glm::mat4(glm::mat3(view)));
        glBindVertexArray(skyboxVAO);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_CUBE_MAP, skyboxTexture);
        glDrawArrays(GL_TRIANGLES, 0, 36);
        glBindVertexArray(0);
        glDepthFunc(GL_LESS);

        // Рендеринг основной сцены
        lightingShader->use();
        lightingShader->setMat4("projection", projection);
        lightingShader->setMat4("view", view);
        lightingShader->setVec3("lightPos", lightPos);
        lightingShader->setVec3("viewPos", camera.Position);
        lightingShader->setVec3("lightColor", lightColor);
        lightingShader->setBool("shadowsEnabled", shadowsEnabled);
        lightingShader->setFloat("alpha", 1.0f);
        if (shadowsEnabled) {
            lightingShader->setMat4("lightSpaceMatrix", lightSpaceMatrix);
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, depthMap);
            lightingShader->setInt("shadowMap", 0);
        }

        // Рендеринг пола
        glm::mat4 model = glm::mat4(1.0f);
        lightingShader->setMat4("model", model);
        lightingShader->setVec3("objectColor", glm::vec3(0.6f, 0.6f, 0.6f));
        glBindVertexArray(floorVAO);
        glDrawArrays(GL_TRIANGLES, 0, 6);

        // Рендеринг кубов
        int cubeIndex = 0;
        for (btRigidBody* rb : rigidBodies) {
            if (!rb || rb == playerBody || rb == floorBody) continue;
            btTransform tr;
            rb->getMotionState()->getWorldTransform(tr);
            btVector3 origin = tr.getOrigin();
            btMatrix3x3 basis = tr.getBasis();
            model = glm::mat4(1.0f);
            model = glm::translate(model, btToGlm(origin));
            glm::mat4 rot(1.0f);
            rot[0][0] = basis[0][0]; rot[1][0] = basis[0][1]; rot[2][0] = basis[0][2];
            rot[0][1] = basis[1][0]; rot[1][1] = basis[1][1]; rot[2][1] = basis[1][2];
            rot[0][2] = basis[2][0]; rot[1][2] = basis[2][1]; rot[2][2] = basis[2][2];
            model *= rot;
            const btCollisionShape* shape = rb->getCollisionShape();
            if (shape->getShapeType() == BOX_SHAPE_PROXYTYPE) {
                const btBoxShape* bx = static_cast<const btBoxShape*>(shape);
                btVector3 he = bx->getHalfExtentsWithMargin();
                model = glm::scale(model, glm::vec3(he.x(), he.y(), he.z()));
            }
            lightingShader->setMat4("model", model);

            // Цвета кубов
            if (cubeIndex == 0) {
                lightingShader->setVec3("objectColor", glm::vec3(0.8f, 0.3f, 0.3f)); // Красный
            }
            else if (cubeIndex == 1) {
                lightingShader->setVec3("objectColor", glm::vec3(0.3f, 0.3f, 0.8f)); // Синий
            }
            else {
                lightingShader->setVec3("objectColor", glm::vec3(0.8f, 0.8f, 0.2f)); // Желтый
            }
            cubeIndex++;

            glBindVertexArray(cubeVAO);
            glDrawArrays(GL_TRIANGLES, 0, 36);
        }

        // Рендеринг желе
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        lightingShader->setFloat("alpha", 0.8f);
        if (jellyCube) {
            model = glm::mat4(1.0f);
            lightingShader->setMat4("model", model);
            lightingShader->setVec3("objectColor", glm::vec3(0.2f, 0.8f, 0.2f));
            if (showWireframe) {
                jellyCube->DrawWireframe();
            }
            else {
                jellyCube->Draw();
            }
        }
        glDisable(GL_BLEND);

        // Рендеринг источника света
        simpleShader->use();
        glm::mat4 lm = glm::mat4(1.0f);
        lm = glm::translate(lm, lightPos);
        lm = glm::scale(lm, glm::vec3(0.2f));
        simpleShader->setMat4("model", lm);
        simpleShader->setMat4("projection", projection);
        simpleShader->setMat4("view", view);
        simpleShader->setVec3("objectColor", lightColor);
        glBindVertexArray(cubeVAO);
        glDrawArrays(GL_TRIANGLES, 0, 36);

        // ImGui интерфейс
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        if (showUI) {
            ImGui::Begin("Flux3D Engine", &showUI, ImGuiWindowFlags_AlwaysAutoResize);

            ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);
            ImGui::Text("Camera: %.1f, %.1f, %.1f", camera.Position.x, camera.Position.y, camera.Position.z);
            ImGui::Text("Objects: %d", (int)boxes.size());

            if (ImGui::CollapsingHeader("Lighting", ImGuiTreeNodeFlags_DefaultOpen)) {
                ImGui::ColorEdit3("Light Color", &lightColor[0]);
                ImGui::DragFloat3("Light Position", &lightPos[0], 0.1f);
                ImGui::Checkbox("Shadows Enabled", &shadowsEnabled);
            }

            if (ImGui::CollapsingHeader("Jelly Physics", ImGuiTreeNodeFlags_DefaultOpen)) {
                ImGui::Checkbox("Wireframe", &showWireframe);
                ImGui::SliderFloat("Stiffness", &jellyStiffness, 10.0f, 2000.0f);
                ImGui::SliderFloat("Damping", &jellyDamping, 0.9f, 1.0f);
                ImGui::SliderFloat("Impulse", &jellyImpulse, 1.0f, 30.0f);
                ImGui::Checkbox("Enable Collision", &enableJellyCollision);

                if (ImGui::Button("Apply Jump Impulse") && jellyCube) {
                    jellyCube->ApplyImpulse(glm::vec3(0.0f, jellyImpulse, 0.0f));
                }
                ImGui::SameLine();
                if (ImGui::Button("Reset Jelly") && jellyCube) {
                    delete jellyCube;
                    jellyCube = new JellyCube(glm::vec3(0.0f, 3.0f, 0.0f), 1.5f);
                    jellyCube->InitializeBuffers();
                }
            }

            if (ImGui::CollapsingHeader("Shooting", ImGuiTreeNodeFlags_DefaultOpen)) {
                ImGui::SliderFloat("Bullet Size", &bulletSize, 0.1f, 1.0f);
                ImGui::SliderFloat("Bullet Force", &bulletForce, 10.0f, 100.0f);
                ImGui::SliderFloat("Bullet Mass", &bulletMass, 0.1f, 5.0f);

                if (ImGui::Button("Shoot (Press E in game)")) {
                    glm::vec3 startPos = camera.Position + camera.Front * 1.5f;
                    btRigidBody* bullet = addDynamicBox(
                        btVector3(bulletSize, bulletSize, bulletSize),
                        glmToBt(startPos),
                        bulletMass
                    );
                    bullet->setRestitution(0.5f);
                    bullet->setFriction(0.3f);
                    glm::vec3 direction = camera.Front;
                    btVector3 impulse = glmToBt(direction * bulletForce);
                    bullet->applyCentralImpulse(impulse);
                }
            }

            ImGui::End();
        }

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    cleanup();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwTerminate();
    return 0;
}