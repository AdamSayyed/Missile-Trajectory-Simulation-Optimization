#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <iomanip>
#include <limits>
#include <string>
#include <utility>
#include <algorithm>

// ---------------------------------------------------------
// Physics & Simulation
// ---------------------------------------------------------

struct State {
    double x, y, z;
    double vx, vy, vz;
    double t;
};

struct MissileParams {
    double m, Cd, A, rho, g, T, burnTime;
    double windX, windZ;
};

// Compute derivatives
State computeAccel(const State& s, double theta, double phi, const MissileParams& p) {
    State d;
    double v = std::sqrt(s.vx*s.vx + s.vy*s.vy + s.vz*s.vz);
    if(v == 0) v = 1e-6; // avoid divide by zero
    
    double D = 0.5 * p.rho * v*v * p.Cd * p.A;
    double currentThrust = (s.t <= p.burnTime) ? p.T : 0.0;
    
    // Forces
    // Drag acts opposite to velocity vector: F_drag = -D * (v_vec / |v|)
    double F_drag_x = -D * (s.vx / v);
    double F_drag_y = -D * (s.vy / v);
    double F_drag_z = -D * (s.vz / v);

    // Thrust vector based on launch angles (simplified: assumes missile points along velocity vector after launch, 
    // but for simple ballistic trajectory we often assume constant thrust direction or thrust along velocity. 
    // Let's assume thrust aligns with velocity for stability, or initial angle if v is small)
    double thrust_x, thrust_y, thrust_z;
    if (s.t <= p.burnTime) {
        if (v > 1.0) {
            thrust_x = p.T * (s.vx / v);
            thrust_y = p.T * (s.vy / v);
            thrust_z = p.T * (s.vz / v);
        } else {
            // Initial thrust direction
            thrust_x = p.T * cos(theta) * cos(phi);
            thrust_y = p.T * sin(theta);
            thrust_z = p.T * cos(theta) * sin(phi);
        }
    } else {
        thrust_x = thrust_y = thrust_z = 0;
    }

    d.vx = (thrust_x + F_drag_x)/p.m + p.windX; // Wind adds to velocity relative to ground? Simplified wind model
    d.vy = (thrust_y + F_drag_y)/p.m - p.g;
    d.vz = (thrust_z + F_drag_z)/p.m + p.windZ;

    d.x = s.vx;
    d.y = s.vy;
    d.z = s.vz;
    d.t = 1.0; 
    return d;
}

// RK4 integrator
State rk4Step(const State& s, double theta, double phi, const MissileParams& p, double dt) {
    State k1 = computeAccel(s, theta, phi, p);
    State s2 = s; 
    s2.x += k1.x*dt/2; s2.y += k1.y*dt/2; s2.z += k1.z*dt/2;
    s2.vx += k1.vx*dt/2; s2.vy += k1.vy*dt/2; s2.vz += k1.vz*dt/2; s2.t += dt/2;

    State k2 = computeAccel(s2, theta, phi, p);
    State s3 = s;
    s3.x += k2.x*dt/2; s3.y += k2.y*dt/2; s3.z += k2.z*dt/2;
    s3.vx += k2.vx*dt/2; s3.vy += k2.vy*dt/2; s3.vz += k2.vz*dt/2; s3.t += dt/2;

    State k3 = computeAccel(s3, theta, phi, p);
    State s4 = s;
    s4.x += k3.x*dt; s4.y += k3.y*dt; s4.z += k3.z*dt;
    s4.vx += k3.vx*dt; s4.vy += k3.vy*dt; s4.vz += k3.vz*dt; s4.t += dt;

    State k4 = computeAccel(s4, theta, phi, p);

    State s_next = s;
    s_next.x += dt/6*(k1.x + 2*k2.x + 2*k3.x + k4.x);
    s_next.y += dt/6*(k1.y + 2*k2.y + 2*k3.y + k4.y);
    s_next.z += dt/6*(k1.z + 2*k2.z + 2*k3.z + k4.z);
    s_next.vx += dt/6*(k1.vx + 2*k2.vx + 2*k3.vx + k4.vx);
    s_next.vy += dt/6*(k1.vy + 2*k2.vy + 2*k3.vy + k4.vy);
    s_next.vz += dt/6*(k1.vz + 2*k2.vz + 2*k3.vz + k4.vz);
    s_next.t += dt;
    return s_next;
}

struct Point3D { double x, y, z; };

std::vector<Point3D> simulateTrajectory(double theta_deg, double phi_deg, const MissileParams& p) {
    double theta = theta_deg * M_PI / 180.0;
    double phi = phi_deg * M_PI / 180.0;
    
    // Initial velocity based on thrust? No, starts at 0 velocity usually, but let's give it a tiny push or just rely on acceleration
    State s {0,0,0, 0,0,0, 0}; 
    
    double dt = 0.01;
    std::vector<Point3D> traj;

    int max_steps = 100000; 
    int steps = 0;

    while(s.y >= 0 && steps < max_steps) {
        traj.push_back({s.x, s.y, s.z});
        s = rk4Step(s, theta, phi, p, dt);
        steps++;
    }
    return traj;
}

// Find optimal launch angle (simplified for 2D plane optimization, ignoring Z for optimization metric)
double findOptimalAngle(const MissileParams& p) {
    double bestAngle = 0;
    double maxRange = -1;
    for(double angle=10; angle<=80; angle+=1.0) {
        auto traj = simulateTrajectory(angle, 0, p); // Optimize for 0 yaw
        if (traj.empty()) continue;
        
        // Horizontal distance in XZ plane
        double dx = traj.back().x;
        double dz = traj.back().z;
        double range = std::sqrt(dx*dx + dz*dz);
        
        if(range > maxRange) {
            maxRange = range;
            bestAngle = angle;
        }
    }
    return bestAngle;
}

// ---------------------------------------------------------
// Visualization
// ---------------------------------------------------------

float camDist = 100.0f;
float camYaw = 0.0f;
float camPitch = 20.0f;
int animStep = 0;
bool paused = false;
int speedMultiplier = 10; // Steps per frame

void renderGround() {
    glBegin(GL_QUADS);
    float gridSize = 2000.0f;
    float step = 100.0f;
    for(float x=-gridSize; x<gridSize; x+=step) {
        for(float z=-gridSize; z<gridSize; z+=step) {
            bool check = ((int)(x/step) + (int)(z/step)) % 2 == 0;
            if(check) glColor3f(0.2f, 0.2f, 0.2f);
            else      glColor3f(0.15f, 0.15f, 0.15f);
            
            glVertex3f(x, 0, z);
            glVertex3f(x+step, 0, z);
            glVertex3f(x+step, 0, z+step);
            glVertex3f(x, 0, z+step);
        }
    }
    glEnd();
    
    // Axes
    glLineWidth(2.0f);
    glBegin(GL_LINES);
    glColor3f(1,0,0); glVertex3f(0,0,0); glVertex3f(100,0,0); // X
    glColor3f(0,1,0); glVertex3f(0,0,0); glVertex3f(0,100,0); // Y
    glColor3f(0,0,1); glVertex3f(0,0,0); glVertex3f(0,0,100); // Z
    glEnd();
}

void renderMissile(const Point3D& p, const Point3D& prevP) {
    glPushMatrix();
    glTranslatef((float)p.x, (float)p.y, (float)p.z);
    
    // Orientation logic
    double dx = p.x - prevP.x;
    double dy = p.y - prevP.y;
    double dz = p.z - prevP.z;
    double mag = std::sqrt(dx*dx + dy*dy + dz*dz);
    
    // Rotate to align with velocity (simplified)
    if (mag > 0) {
        // Normalize
        dx /= mag; dy /= mag; dz /= mag;
        
        // Pitch (angle with XZ plane)
        double pitch = asin(dy) * 180.0 / M_PI;
        // Yaw (angle in XZ plane)
        double yaw = atan2(dx, dz) * 180.0 / M_PI;
        
        glRotatef((float)yaw, 0, 1, 0);
        glRotatef((float)-pitch, 1, 0, 0);
    }

    // Draw Missile Body (Grey Cylinder-ish)
    glColor3f(0.7f, 0.7f, 0.7f);
    float r = 0.2f; // Radius
    float len = 1.5f; // Length
    
    glBegin(GL_QUADS);
    // Body
    glVertex3f(-r, -r, -len/2); glVertex3f(r, -r, -len/2); glVertex3f(r, r, -len/2); glVertex3f(-r, r, -len/2);
    glVertex3f(-r, -r, len/2); glVertex3f(r, -r, len/2); glVertex3f(r, r, len/2); glVertex3f(-r, r, len/2);
    glVertex3f(-r, -r, -len/2); glVertex3f(-r, -r, len/2); glVertex3f(-r, r, len/2); glVertex3f(-r, r, -len/2);
    glVertex3f(r, -r, -len/2); glVertex3f(r, -r, len/2); glVertex3f(r, r, len/2); glVertex3f(r, r, -len/2);
    glVertex3f(-r, r, -len/2); glVertex3f(r, r, -len/2); glVertex3f(r, r, len/2); glVertex3f(-r, r, len/2);
    glVertex3f(-r, -r, -len/2); glVertex3f(r, -r, -len/2); glVertex3f(r, -r, len/2); glVertex3f(-r, -r, len/2);
    glEnd();
    
    // Fins (Red)
    glColor3f(0.8f, 0.2f, 0.2f);
    glBegin(GL_TRIANGLES);
    glVertex3f(0, 0, -len/2); glVertex3f(0, r*3, -len/2 - 0.5f); glVertex3f(0, 0, -len/2 + 0.5f);
    glVertex3f(0, 0, -len/2); glVertex3f(0, -r*3, -len/2 - 0.5f); glVertex3f(0, 0, -len/2 + 0.5f);
    glVertex3f(0, 0, -len/2); glVertex3f(r*3, 0, -len/2 - 0.5f); glVertex3f(0, 0, -len/2 + 0.5f);
    glVertex3f(0, 0, -len/2); glVertex3f(-r*3, 0, -len/2 - 0.5f); glVertex3f(0, 0, -len/2 + 0.5f);
    glEnd();

    glPopMatrix();
}

void renderExplosion(const Point3D& p, float scale) {
    glPushMatrix();
    glTranslatef((float)p.x, (float)p.y, (float)p.z);
    glScalef(scale, scale, scale);
    
    glBegin(GL_TRIANGLES);
    // Debris/Smoke (Dark Grey/Brown) - Less fiery
    glColor3f(0.4f, 0.4f, 0.4f); 
    for(int i=0; i<20; ++i) {
        float x = (float)(rand()%100 - 50)/60.0f; // More compact
        float y = (float)(rand()%100 - 50)/60.0f;
        float z = (float)(rand()%100 - 50)/60.0f;
        glVertex3f(0,0,0);
        glVertex3f(x,y,z);
        glVertex3f(x*0.5f, y+0.5f, z*0.5f);
    }
    // Core (Dull Orange)
    glColor3f(0.8f, 0.4f, 0.0f); 
    for(int i=0; i<10; ++i) {
        float x = (float)(rand()%100 - 50)/100.0f;
        float y = (float)(rand()%100 - 50)/100.0f;
        float z = (float)(rand()%100 - 50)/100.0f;
        glVertex3f(0,0,0);
        glVertex3f(x,y,z);
        glVertex3f(x*0.5f, y+0.2f, z*0.5f);
    }
    glEnd();
    glPopMatrix();
}

void renderTrajectory(const std::vector<Point3D>& traj, int currentStep) {
    // Draw full path faintly
    glLineWidth(1.0f);
    glColor3f(0.3f, 0.3f, 0.3f);
    glBegin(GL_LINE_STRIP);
    for(const auto& pt : traj) glVertex3f((float)pt.x, (float)pt.y, (float)pt.z);
    glEnd();

    // Draw active path brightly
    glLineWidth(3.0f);
    glBegin(GL_LINE_STRIP);
    for(int i=0; i<=currentStep && i<traj.size(); ++i) {
        float t = (float)i / traj.size();
        glColor3f(1.0f - t, t, 0.2f);
        glVertex3f((float)traj[i].x, (float)traj[i].y, (float)traj[i].z);
    }
    glEnd();
}

void processInput(GLFWwindow* window) {
    if(glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
        
    if(glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) camYaw -= 1.0f;
    if(glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) camYaw += 1.0f;
    if(glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) camPitch += 1.0f;
    if(glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) camPitch -= 1.0f;
    if(glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) camDist -= 1.0f;
    if(glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS) camDist += 1.0f;
    
    // Animation controls
    static bool pPressed = false;
    if(glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS) {
        if(!pPressed) { paused = !paused; pPressed = true; }
    } else pPressed = false;

    if(glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS) animStep = 0;
}

int main() {
    MissileParams p;
    std::cout << "--- Missile Trajectory 3D Simulation ---\n";
    
    std::cout << "Enter missile mass (kg, default 50): "; 
    if (std::cin.peek() == '\n') { std::cin.ignore(); p.m = 50; } else { std::cin >> p.m; std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); }

    std::cout << "Enter thrust T (N, default 1000): "; 
    if (std::cin.peek() == '\n') { std::cin.ignore(); p.T = 1000; } else { std::cin >> p.T; std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); }

    std::cout << "Enter burn time (s, default 10): "; 
    if (std::cin.peek() == '\n') { std::cin.ignore(); p.burnTime = 10.0; } else { std::cin >> p.burnTime; std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); }

    std::cout << "Enter wind X (m/s, default 0): "; 
    if (std::cin.peek() == '\n') { std::cin.ignore(); p.windX = 0; } else { std::cin >> p.windX; std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); }

    std::cout << "Enter wind Z (m/s, default 0): "; 
    if (std::cin.peek() == '\n') { std::cin.ignore(); p.windZ = 0; } else { std::cin >> p.windZ; std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); }

    p.Cd = 0.3; p.A = 0.1; p.rho = 1.225; p.g = 9.81;

    std::cout << "\nCalculating optimal angle...\n";
    double theta_opt = findOptimalAngle(p);
    std::cout << "Optimal Launch Angle: " << theta_opt << " degrees\n";

    std::cout << "Simulating trajectory...\n";
    auto traj = simulateTrajectory(theta_opt, 0, p);
    std::cout << "Simulation complete. Points: " << traj.size() << "\n";
    std::cout << "Max Distance: " << std::sqrt(traj.back().x*traj.back().x + traj.back().z*traj.back().z) << " m\n";

    // OpenGL Init
    if(!glfwInit()) {
        std::cerr << "Failed to initialize GLFW\n";
        return -1;
    }

    GLFWwindow* window = glfwCreateWindow(1024, 768, "Missile Trajectory 3D", nullptr, nullptr);
    if(!window) { glfwTerminate(); return -1; }
    glfwMakeContextCurrent(window);

    std::cout << "\nControls:\n";
    std::cout << "  W/S: Pitch Camera\n";
    std::cout << "  A/D: Yaw Camera\n";
    std::cout << "  Q/E: Zoom In/Out\n";
    std::cout << "  P: Pause/Resume Animation\n";
    std::cout << "  R: Restart Animation\n";
    std::cout << "  ESC: Exit\n";

    while(!glfwWindowShouldClose(window)) {
        processInput(window);

        // Update Animation
        if (!paused && !traj.empty()) {
            animStep += speedMultiplier;
            if (animStep >= traj.size()) animStep = traj.size() - 1;
        }

        int width, height;
        glfwGetFramebufferSize(window, &width, &height);
        glViewport(0, 0, width, height);
        glClearColor(0.05f, 0.05f, 0.1f, 1.0f); // Dark blueish sky
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective(45.0, (double)width/height, 0.1, 50000.0);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        
        // Camera logic
        Point3D target = traj.empty() ? Point3D{0,0,0} : traj[animStep]; // Follow missile?
        // Let's look at the missile!
        // Or keep looking at center? Looking at missile is more dynamic.
        // Let's stick to looking at the midpoint of the whole trajectory for stability, but maybe pan slightly.
        
        float midX = traj.back().x / 2.0f;
        float midZ = traj.back().z / 2.0f;
        
        float radYaw = camYaw * M_PI / 180.0f;
        float radPitch = camPitch * M_PI / 180.0f;
        float camX = midX + camDist * cos(radPitch) * sin(radYaw);
        float camY = camDist * sin(radPitch);
        float camZ = midZ + camDist * cos(radPitch) * cos(radYaw);
        
        if (camY < 1.0f) camY = 1.0f; // Don't go underground

        gluLookAt(camX, camY, camZ, midX, 0, midZ, 0, 1, 0);

        renderGround();
        renderTrajectory(traj, animStep);
        
        if (!traj.empty()) {
            Point3D prev = (animStep > 0) ? traj[animStep-1] : traj[0];
            renderMissile(traj[animStep], prev);
            
            // Explosion at the end
            if (animStep >= traj.size() - 1) {
                renderExplosion(traj.back(), 2.5f);
            }
        }

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
