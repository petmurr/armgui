#include "imgui.h"

#include <cmath>            // for pendulum
#include <cstdint>          // for knob
#include <string>           // for dynamixel usb devicename
#include <array>            // for splining + joint arrays
#include <Eigen/Dense>      // for quintic spline solving

#include "get_ports.h"      // for getting ports
#include "use_dxl.h"        // for interacting with dynamixel sdk

#define BUFFER_SIZE 35
#define IM_PI 3.14159265358979323846f

/*
gui.cpp
main.cpp is where imgui does its magic + setup. gui.cpp is where armgui mostly lives.
functions that begin with "show" are imgui windows. 
*/ 

/* calcCoeff()
Calculates the quintic polynomial coefficients for boundary conditions pos, vel, acc @ t0 and pos, vel acc @ tf
*/
std::array<double, 6> calcCoeff(std::array<double, 8> &bc) {
    //                               (@ t0)  (@ tf)
    // boundary conditions: {t0, tf, p v a , p v a}

    Eigen::MatrixXd A(6,6); // defines 6x6 matrix
    Eigen::VectorXd b(6);   // defines 6x1 vector

    A << 1, bc[0], pow(bc[0], 2),   pow(bc[0], 3),     pow(bc[0], 4),     pow(bc[0], 5),
         0,     1,       2*bc[0], 3*pow(bc[0], 2),   4*pow(bc[0], 3),   5*pow(bc[0], 4),
         0,     0,             2,         6*bc[0],  12*pow(bc[0], 2),  20*pow(bc[0], 3),
         1, bc[1], pow(bc[1], 2),   pow(bc[1], 3),     pow(bc[1], 4),     pow(bc[1], 5),
         0,     1,       2*bc[1], 3*pow(bc[1], 2),   4*pow(bc[1], 3),   5*pow(bc[1], 4),
         0,     0,             2,         6*bc[1],  12*pow(bc[1], 2),  20*pow(bc[1], 3);

    // b << pi,    vi,    ai,    pf,    vf,    af;
    b << bc[2], bc[3], bc[4], bc[5], bc[6], bc[7];

    Eigen::VectorXd coeff = A.colPivHouseholderQr().solve(b);

    // std::cout << "The coefficients are:" << std::endl;
    // std::cout << coeff << std::endl;

    std::array<double, 6> coeff_array;
    for (int i = 0; i < 6; ++i) {
        coeff_array[i] = coeff(i);  // Note: use parenthesis for Eigen's operator()
    }
    
    return coeff_array;
}

/* calcCurrentPVA() 
Calculates the position, velocity, and acceleration of the trajectory given it's coeff_array (see calcCoeff) and the current time.
*/
std::array<double, 3> calcCurrentPVA(std::array<double, 6>& coeffs, double t) {

    double p = coeffs[0] +   coeffs[1]*t +    coeffs[2]*(pow(t, 2)) +    coeffs[3]*(pow(t, 3)) +   coeffs[4]*(pow(t, 4)) + coeffs[5]*(pow(t, 5)); 
    double v = coeffs[1] + 2*coeffs[2]*t +  3*coeffs[3]*(pow(t, 2)) +  4*coeffs[4]*(pow(t, 3)) + 5*coeffs[5]*(pow(t, 4));
    double a = coeffs[2] + 6*coeffs[3]*t + 12*coeffs[4]*(pow(t, 2)) + 20*coeffs[5]*(pow(t, 3));
    // for (int i=0; i<6; i++) {
    //     printf("coeff_array: %f (time: %f)\n", coeffs[i], t);
    // }

    // printf("=============== t: %f\tp: %f\t v: %f\ta: %f \n", t, p, v, a);
    std::array<double, 3> pva_array = {p, v, a};
    return pva_array;
}

/* 
createKnob()
creates an interactive dial indicator with two needles:
needle1:    our setpoint, primary needle
needle2:    our measured dynamixel position, secondary needle
p:          ImVec2, (x,y) in pixels of where our window starts on screen 
knob_loc    ImVec2, (x,y) in pixels of where our knob is.
lim_min     
lim_max     
*/
int32_t createKnob(int32_t needle1, int32_t needle2, const ImVec2 &p, float scale, bool &knob_is_enabled, float lim_min, float lim_max, float deg_min, float deg_max) { 
    
    ImDrawList* draw_list = ImGui::GetWindowDrawList();             // Imgui backend for rendering shapes

    // define colors
    static const ImU32 col_white    = IM_COL32(190, 200, 200, 255);   
    static const ImU32 col_blue     = IM_COL32(0, 100, 160, 255);    
    static const ImU32 col_gray     = IM_COL32(133, 133, 133, 255);  
    
    static ImU32 col_needle1;
    static ImU32 col_needle2;
    static ImU32 col_bg;

    if (knob_is_enabled) { // This is to be able to gray out the knob when it's not enabled yet.
        col_needle1 = col_gray;
        col_needle2 = col_white;
        col_bg = col_blue;
    } else {
        col_needle1 = col_gray;
        col_needle2 = col_white;
        col_bg = col_gray;
    }
    
    // position the knob to the center right of the widget
    // float center_x = p.x + knob_loc[0]; 
    // float center_y = p.y + knob_loc[1]; 
    float center_x = p.x + (ImGui::GetWindowWidth() * 0.85f);
    float center_y = p.y + (ImGui::GetWindowHeight() * 0.36f);
    static float needle_thickness = scale * (6.0f);
    float theta_measured = needle2 * -2 * IM_PI * 0.000244140625; // 1/4096 = 0.0002 
    float theta_set = needle1 * -2 * IM_PI * 0.000244140625; 
    
    // Calculate where the end of our line needs to be drawn
    ImVec2 pos_measured = ImVec2(90 * cos(theta_measured), 90 * sin(theta_measured)); 
    ImVec2 pos_set = ImVec2(90 * cos(theta_set), 90 * sin(theta_set)); 

    // ImGui::Text("deg equ: %.1f", theta_measured*57.29578); // debug
    
    float posX = center_x + scale*(pos_measured.x);
    float posY = center_y + scale*(pos_measured.y);
    float posX_set = center_x + scale*(pos_set.x);
    float posY_set = center_y + scale*(pos_set.y);

    ImVec2 pos_limit_min = ImVec2(center_x + (90 * cos(-40.0f* (1/57.29578))), center_y + (90 * sin(-40.0f * (1/57.29578))));
    ImVec2 pos_limit_max = ImVec2(center_x + (90 * cos(55.0f* (1/57.29578))), center_y + (90 * sin(55.0f * (1/57.29578))));

    // Draw our knob with two circles and two lines
    draw_list->AddCircleFilled(ImVec2(center_x, center_y), scale * (100.0f), col_bg);           // background circle
    draw_list->AddLine( ImVec2(center_x, center_y),                                             // user-positioned needle. requested position of servo.
                        ImVec2(posX_set, posY_set),
                        col_needle1, needle_thickness);
    draw_list->AddLine( ImVec2(center_x, center_y),                                             // measured needle. actual position of servo.
                        ImVec2(posX, posY), 
                        col_needle2, needle_thickness);

    static float th = 15.0f;
    // ImGui::SliderFloat("thickness", &th, 0.0, 20.0f);

    draw_list->PathArcTo(ImVec2(center_x, center_y), 50.0f, lim_min *(2 * IM_PI * 0.000244140625), lim_max *(2 * IM_PI * 0.000244140625), 0); // int32 -> radians 
    draw_list->PathStroke(col_gray, ImDrawFlags_None, th);
    draw_list->AddCircleFilled(ImVec2(center_x, center_y), scale * (100.0f - 50.0f), col_bg);   // circle on top of arrangement, hides the center of the needles.

    // mouse interaction
    const ImVec2 p_before_button = ImGui::GetCursorScreenPos(); // save this so we can return to it and add things to the bottom of the widget
    ImGui::SetCursorScreenPos(ImVec2(center_x - (scale*100), center_y - (scale*100)));
    ImGui::InvisibleButton("knob", ImVec2(scale * (200.0f), scale * (200.0f)));

    ImGui::SetCursorScreenPos(p_before_button); // reset cursor position

    // Check if the knob is active (being dragged)
    if (ImGui::IsItemActive() && knob_is_enabled) {
        /* Vertical knob control: 
            turn the knob by dragging upwards and downwards. */
        
        /*
        // Get the change in mouse position through ImGui
        ImVec2 drag_delta = ImGui::GetMouseDragDelta(ImGuiMouseButton_Left);
        
        // Change the sensitivity of the input by scaling the vertical component
        int32_t delta_input = -10 * drag_delta.y;
        
        // Change the output.
        needle1 += delta_input;
        
        // Reset mouse drag delta so it accumulates correctly (for vertical knob control)
        ImGui::ResetMouseDragDelta(ImGuiMouseButton_Left);
        */

        /* 'Direct' knob control: 
            Turn the knob continously by clicking and dragging in a circle. Knob turns as if locked onto mouse.*/
    
        // Get mouse position.
        ImVec2 mouse_pos = ImGui::GetMousePos(); 
        
        // Get mouse position relative to our knob's center.
        ImVec2 mouse_pos_rel = ImVec2(mouse_pos.x - center_x, mouse_pos.y - center_y); 
        
        // Calculate the angle of our mouse relative to our knob (degrees).
        double theta = 57.29578*atan2(mouse_pos_rel.y, mouse_pos_rel.x);

        // create a static variable to calculate the change in angle over each frame 
        static double last_relative_theta = theta;
        double delta_theta = theta - last_relative_theta;

        // debugging
        // printf("delta_theta= current (%.1f) - last (%.1f) = %.1f\n", relative_theta, last_relative_theta, delta_theta);
        
        // Handle looping. Our theta value is limited to -180 > theta >= +180, so correct delta_theta at the transition.
        if (delta_theta > 180 || delta_theta < -180) {
            delta_theta < 0 ? delta_theta += 360 : delta_theta -= 360;
        }

        // Handle repositioning. and crazy fast users. If the change is too great, just reset it. 
        if (delta_theta > 22.0 || delta_theta < -22.0) {
            delta_theta = 0.0;
        }

        // if (((lim_min + 2020) < needle1) && ((lim_max + 2020) > needle1)) {
        //     needle1 += delta_theta * 11.377; // Sensitivity. At 11.377, knob and mouse input is 1:1. 4096 ticks / 360deg = 11.3777
        // } else {
        //     needle1 += delta_theta * 0.1;
        //     printf("\n\nslowing down...\n\n");
        // }
        
        if ((deg_min * (11.377777778)) > (float)needle1) {
            needle1 +=  5;
        } else if ((deg_max * (11.377777778)) < (float)needle1) {
            needle1 -=  5;
        } else {
            needle1 -= delta_theta * 11.377; // flipped to make CCW
        }

        last_relative_theta = theta;
    }

    return needle1;
}

/* showServoWindow() 
Provides a visual interface to control position of a single servo. 
As of now, we're using a moving average to convert a user's input to an encoder position we send 
to our dynamixel
title:      c style string title of window (use joint name)
input:      direct user input (setpoint)
output:     our moving average
measured:   measured encoder position as reported by dynamixel 
*/
int32_t showServoWindow(const char* title, 
                        int32_t input, 
                        int32_t output, 
                        int32_t measured, 
                        bool knob_is_enabled, 
                        float lim_min, float lim_max, 
                        float deg_min, float deg_max,
                        uint8_t opmode) {

    // static float knob_xy[2] = {415.0f, 55.0f}; // i know we're throwing around ImVec2 and pos_x and pos_y but sliderfloat2 needs a float array
    // float knob_xy[2] = {ImGui::GetWindowWidth()*0.8f, ImGui::GetWindowHeight()*0.5};
    static float scale = 0.58f;
    
    float degree_equivalent = ((float)measured * 2 * IM_PI * 0.000244140625)*57.29578;

    ImGui::Begin(title);

    const ImVec2 p_start = ImGui::GetCursorScreenPos(); // this is at the beginning, so p is located at top of widget (px, px)
    ImGui::PushItemWidth(200); // shrink our sliders (enforce size)
    ImGui::SliderInt("input", &input, -3000, 3000, "%d ticks");
    
    ImGui::BeginDisabled();
    // ImGui::SliderInt("output", &output, -3000, 3000);
    ImGui::SliderFloat("degree_equivalent", &degree_equivalent, -180, 180, "%.1f deg");
    ImGui::SliderInt("measured", &measured, -3000, 3000, "%d ticks");
    ImGui::EndDisabled();

    const std::array<std::string, 17> opmode_names = {  
        "Current control", 
        "Velocity control",
        "", 
        "Position control", 
        "Extended Position control", 
        "Current-based Position control", 
        "", "", "", "", "", "", "", "", "", "",
        "PWM Control"
        };

    if (opmode >= 0 && opmode < opmode_names.size() && !opmode_names[opmode].empty()) {
        ImGui::Text("Operating mode: %s", opmode_names[opmode].c_str());
    } else {
        ImGui::Text("Operating mode: Undefined");
    }   

    // for positioning
    // ImGui::SliderFloat("scale", &scale, 0.1f, 3.0f);
    // ImGui::SliderFloat2("knob_loc", knob_xy, 0.0f, 1000.0f);

    // ImGui::Checkbox("knob_is_enabled", &knob_is_enabled);

    // static float lim_min = 1000;
    // static float lim_max = 2000;
    // ImGui::SliderFloat("lim_min", &lim_min, -3000, 3000);
    // ImGui::SliderFloat("lim_max", &lim_max, -3000, 3000);

    input = createKnob(input, measured, p_start, scale, knob_is_enabled, lim_min, lim_max, deg_min, deg_max);

    ImGui::End();
    
    return input;
}


/* drawSettingsWindow 
provides an interface to adjust visual and actual joint limits, scale the UI, and toggling vsync
and changing operating mode
*/
void drawSettingsWindow(ImGuiIO &io, 
                        bool& enable_vsync,
                        float(&visual_limits)[5][2],
                        float(&literal_limits)[5][2]) {

    ImGui::Begin("Settings");

    // Frame times
    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
    ImGui::Text("Sluggish arm? Try disabling vsync:");
    ImGui::Checkbox("enable_vsync", &enable_vsync);

    // Control UI size
    ImGui::Text("Scale UI:");
    static float  ui_scale = 1.0;
    io.FontGlobalScale = ui_scale;
    ImGui::SliderFloat("ui_scale", &ui_scale, 1.0f, 3.0f);

    // Visual joint limits (encoder values, 0 = 0 deg, 4098 = 360 deg)
    ImGui::Text("Set visual joint limits:");
    ImGui::SliderFloat2("j0 min, max visual", visual_limits[0], 0.0f, 2*4098.0f);
    ImGui::SliderFloat2("j1 min, max visual", visual_limits[1], 0.0f, 2*4098.0f);
    ImGui::SliderFloat2("j2 min, max visual", visual_limits[2], 0.0f, 2*4098.0f);
    ImGui::SliderFloat2("j3 min, max visual", visual_limits[3], 0.0f, 2*4098.0f);
    ImGui::SliderFloat2("j4 min, max visual", visual_limits[4], 0.0f, 2*4098.0f);

    // Literal joint limits (deg)
    ImGui::Text("Set literal joint limits");
    ImGui::SliderFloat2("j0 min, max literal", literal_limits[0], 0.0f, 360.0f);
    ImGui::SliderFloat2("j1 min, max literal", literal_limits[1], 0.0f, 360.0f);
    ImGui::SliderFloat2("j2 min, max literal", literal_limits[2], 0.0f, 360.0f);
    ImGui::SliderFloat2("j3 min, max literal", literal_limits[3], 0.0f, 360.0f);
    ImGui::SliderFloat2("j4 min, max literal", literal_limits[4], 0.0f, 360.0f);

    ImGui::End();
}

void drawArmSkeleton(std::array<float, 5> joint_array) {
    ImGui::Begin("Arm Skeleton");
    // printf("jointarray0", joint_array[0]);
    static float scale = 1.0f * ImGui::GetWindowDpiScale();
    float visual_offset[2] = {ImGui::GetWindowWidth() * 0.5f, ImGui::GetWindowHeight() * 0.5f};
    static float joint_offsets[5] = {0.0f, IM_PI/2, -IM_PI/2, IM_PI, 0.0f};

    for (int i = 0; i<5; i++) {
        joint_array[i] = joint_array[i] + joint_offsets[i];
    }

    // joint 0 is not shown
    // joint 1 is fixed
    ImVec2 p = ImGui::GetCursorScreenPos();
    ImVec2 j1 = ImVec2(p.x + visual_offset[0], p.y + visual_offset[1]);

    // joint 2 
    float l1 = scale * 50.0f;
    ImVec2 j2 = ImVec2(j1.x + l1 * cos(joint_array[1]),
                       j1.y + l1 * sin(joint_array[1]));

    // joint 3
    float l2 = scale * 50.0f;
    ImVec2 j3 = ImVec2(j2.x + l2 * cos(joint_array[1] + joint_array[2]),
                       j2.y + l2 * sin(joint_array[1] + joint_array[2]));

    // joint 4
    float l3 = scale * 50.0f;
    ImVec2 j4 = ImVec2(j3.x + l3 * cos(joint_array[1] + joint_array[2] + joint_array[3]),
                       j3.y + l3 * sin(joint_array[1] + joint_array[2] + joint_array[3]));

    // joint 5 (end effector)
    // float l4 = scale * 50.0f;
    // ImVec2 j5 = ImVec2(j4.x + l4 * cos(joint_array[1] + joint_array[2] + joint_array[3] + joint_array[4]),
    //                    j4.y + l4 * sin(joint_array[1] + joint_array[2] + joint_array[3] + joint_array[4]));

    ImDrawList* draw_list = ImGui::GetWindowDrawList();

    // Draw links
    draw_list->AddLine(j1, j2, IM_COL32(255, 0, 0, 255), 3.0f);
    draw_list->AddLine(j2, j3, IM_COL32(0, 255, 0, 255), 3.0f);
    draw_list->AddLine(j3, j4, IM_COL32(0, 0, 255, 255), 3.0f);
    
    // Draw joints
    draw_list->AddCircleFilled(j1, 5.0f, IM_COL32(255, 255, 255, 255));
    draw_list->AddCircleFilled(j2, 5.0f, IM_COL32(255, 255, 255, 255));
    draw_list->AddCircleFilled(j3, 5.0f, IM_COL32(255, 255, 255, 255));

    // Settings
    /*
    ImGui::SliderFloat("Scale", &scale, 0.5f, 13.0f);
    ImGui::SliderFloat2("Offset x, y", visual_offset, 0.0f, 1900.0f);
    // ImGui::SliderFloat("joint 1 offset", &joint_offsets[0], -(2*IM_PI), (2*IM_PI)); // unimportant
    ImGui::SliderFloat("joint 2 offset", &joint_offsets[1], -(2*IM_PI), (2*IM_PI)); // +90, pi/4
    ImGui::SliderFloat("joint 3 offset", &joint_offsets[2], -(2*IM_PI), (2*IM_PI)); // -90, -pi/4
    ImGui::SliderFloat("joint 4 offset", &joint_offsets[3], -(2*IM_PI), (2*IM_PI)); // + 180, 2pi
    // ImGui::SliderFloat("joint 5 offset", &joint_offsets[4], -(2*IM_PI), (2*IM_PI)); // unimportant
    */
    
    ImGui::End();
}


/* 
drawStatusWindow
Provides a visual interface to see and initiate program functions.
*/
const char* drawStatusWindow(   int& primary_dxl_id, 
                                int bauds_current_idx, 
                                bool &is_connected, 
                                bool &is_homed, 
                                bool &is_connection_requested,
                                bool &is_torque_requested,
                                bool &is_home_requested,
                                bool &is_stow_requested,
                                bool &is_disconnect_requested,
                                dynamixel::PacketHandler* packetHandler,
                                dynamixel::PortHandler* portHandler,
                                std::array<uint8_t, 5> &opmodes) {

    ImGui::Begin("armgui status");

    // ImGui::PushItemWidth(400); // shrink our sliders (enforce size)
    
    ImGui::Text("Make sure your arm is powered!");

    if (ImGui::Button("Connect", ImVec2(400, 50))) {
        is_connection_requested = is_connection_requested ? false : true;
    }
    if (ImGui::Button("Home", ImVec2(198, 100))) {

        for (int i = 0; i<5; i++) {
            if (opmodes[i] != 3) {
                ImGui::OpenPopup("Operating modes incorrect!");
            } else {
                is_torque_requested = is_torque_requested ? false : true; 
            }
        }
    }
    ImGui::SameLine(0.0f,4.0f);
    if (ImGui::Button("Stow", ImVec2(198, 100))) {
        is_stow_requested = is_stow_requested ? false : true;
    }

    if (ImGui::Button("Stow & Disconnect", ImVec2(400, 50))) {
        is_disconnect_requested = is_disconnect_requested ? false : true;
    }

    /*
    // Display and select baud rates 
    ImGui::Text("RBE 3001 uses a baud rate of 1000000 and dynamixel IDs 11-15. These are here for visualization & future work."); // TODO: ability to scan through options.
    
    const char* bauds[] = {"9600", "57600", "115200", "1000000", "2000000", "3000000", "4000000", "4500000"};
    const char* bauds_preview_value = bauds[bauds_current_idx];
    if (ImGui::BeginCombo("baud rate", bauds_preview_value)) {
        for (int n = 0; n < IM_ARRAYSIZE(bauds); n++) {
            const bool is_selected = (bauds_current_idx== n);
            if (ImGui::Selectable(bauds[n], is_selected)) bauds_current_idx = n;
            if (is_selected) ImGui::SetItemDefaultFocus();
        }
        ImGui::EndCombo();
    }
    */

    if (ImGui::BeginPopupModal("Operating modes incorrect!")) {
        for (int i = 0; i < 5; i++) {
            if (opmodes[i] != 3) {
                ImGui::Text("Dynamixel id %d is in operating mode %d instead of 3 (position)!", 11 + i, opmodes[i]);
            }
        }
        ImGui::Text("Set motor(s) to position control to continue:");
    
        if (ImGui::Button("set all motors to position control", ImVec2(300, 50))) {
            for (int i = 0; i < 5; i++) {
                setOpMode(portHandler, packetHandler, 11+i, 3);
            }
            opmodes = {3, 3, 3, 3, 3};

            ImGui::CloseCurrentPopup();
        }
    
    ImGui::EndPopup();
    }
    
    
    // === ports/devices
    ImGui::Text("devices available:");
    
    // logic to continually look for ports and convert those strings into cstrings for imgui's combo box (dropdown)
    // init NEW empty const char* list we will fill with whatever pops up from our helper function listPorts()
    static std::vector<std::string> ports = {"No ports available!"};
    static int counter = 0;
    static const char** ports_cstr = nullptr;  // move ports_cstr to static for reuse
    static int ports_current_idx = 0; // selected port within list

    if ((counter % 500) == 0) {
        ports = listPorts();

        // Deallocate previous ports_cstr array
        if (ports_cstr) {
            delete[] ports_cstr;
        }

        // Allocate new ports_cstr array
        ports_cstr = new const char*[ports.size()];

        for (int i = 0; i < ports.size(); i++) {
            ports_cstr[i] = ports[i].c_str(); // convert std::string to c style string
        }

        // Ensure ports_current_idx is within bounds
        if (ports_current_idx >= ports.size()) {
            ports_current_idx = 0;
        }
    }
    counter += 1;

    const char* ports_preview_value = ports_cstr[ports_current_idx];

    if (ImGui::BeginCombo("ports", ports_preview_value)) {
        for (int n = 0; n < ports.size(); n++) {
            const bool is_selected = (ports_current_idx == n);
            if (ImGui::Selectable(ports_cstr[n], is_selected)) ports_current_idx = n;
            if (is_selected) ImGui::SetItemDefaultFocus();
        }
        ImGui::EndCombo();
    }

    ImGui::End();

    return ports_cstr[ports_current_idx];
}

/* drawStatsBoxes()
#TODO desc and better name
*/ 
void drawStatusBoxes(bool &is_connected, bool &is_torqued, bool &is_homed) {
    // Begin a new window
    ImGui::Begin("Status Boxes");

    // Draw the "Connection" status box
    ImGui::Text("Connection Status:");
    ImVec2 p = ImGui::GetCursorScreenPos();
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    ImU32 col = is_connected ? IM_COL32(0, 255, 0, 255) : IM_COL32(255, 0, 0, 255);
    draw_list->AddRectFilled(p, ImVec2(p.x + 50, p.y + 50), col);
    ImGui::Dummy(ImVec2(50, 50)); // Add dummy to move cursor position

    // Draw the "Torque" status box
    ImGui::Text("Torque Status:");
    p = ImGui::GetCursorScreenPos();
    col = is_torqued ? IM_COL32(0, 255, 0, 255) : IM_COL32(255, 0, 0, 255);
    draw_list->AddRectFilled(p, ImVec2(p.x + 50, p.y + 50), col);
    ImGui::Dummy(ImVec2(50, 50)); // Add dummy to move cursor position

    // Draw the "Torque" status box
    ImGui::Text("Homed Status:");
    p = ImGui::GetCursorScreenPos();
    col = is_homed ? IM_COL32(0, 255, 0, 255) : IM_COL32(255, 0, 0, 255);
    draw_list->AddRectFilled(p, ImVec2(p.x + 50, p.y + 50), col);
    ImGui::Dummy(ImVec2(50, 50)); // Add dummy to move cursor position

    // End the window
    ImGui::End();
}

int32_t calcAvg(int32_t arr[BUFFER_SIZE]) { 
    int32_t sum = 0;
    for (int i=0; i < BUFFER_SIZE; i++) {
        sum += arr[i];
    }
    return BUFFER_SIZE > 0 ? sum / BUFFER_SIZE : 0;
}

enum State {
    STATE_DISCONNECTED,             // (loops)  starting state
    STATE_CONNECTING,               // (single) looking for dynamixels
    STATE_CONNECTED_ONLY,           // (loops)  connected, no torque, no home
    STATE_CONNECTED_TORQUE_ONLY,    // (single) connected, yes torque, no home. Immediately followed by handle home request.
    STATE_CONNECTED_TORQUE_HOME,    // (loops)  connected, yes torque, yes home
    STATE_HANDLE_TORQUE_REQUEST,    // (single) in the process of enabling the torque (as triggered by UI). Only works if in STATE_CONNECTED_ONLY state
    STATE_HANDLE_HOME_REQUEST,      // (single) in the process of homing the arm (as trigged by UI). Only works if in STATE_CONNECTED_TORQUE_ONLY state
    STATE_MOVE_TO_HOME              // (loops)  moves arm to "homed" position w/ calculated spline
};

/* [gui.cpp] foremost call from main.cpp. 
for each frame, render all our tools and talk to our dynamixels.
*/
void update(ImGuiIO& io, bool& enable_vsync) { 

    // State machine variables
    static State current_state = STATE_DISCONNECTED;    // holds enum State, the current state of armgui, as seen by switch/case blocks below
    static bool is_connected = false;                   // arm is connected. keep reading/writing joint positions.
    static bool is_torqued = false;                     // if torque has been enabled.
    static bool is_homed = false;                       // if the robot has been homed. enables user control.
    static bool is_torque_requested = false;            // user requests torque be enabled.
    static bool is_home_requested = false;              // user requests the robot to be homed. enables torque and user input
    static bool is_connection_requested = false;        // user requests connection. runs a block of code once.
    static bool is_stow_requested = false;              // user requests arm to be stowed so torque may be disabled w/o crashing
    static bool is_disconnect_requested = false;        // user requests disconnection. Stow and disable torque.
    
    // Qunitic splining variables
    static std::array<int32_t, 5> home_pos = {2048, 2048, 2048, 2048, 2048};    // homed position. can change if you want splining to another location.
    static std::array<std::array<double, 6>, 5> joints_coeffs;                  // 5 array of coefficients (each w/ 6 vals) needed by calcPVA for determining current pos, vel, acc.
    static double current_time;
    static bool is_homing = false;                                              // while the robot is homing. this is because we need to calculate quintic spline during positioning.
    static double time_end;
    static double time_start;                       // 
    static bool is_coeff_calcd = false;             // to calc the homing coefficients once.
    static bool is_ma_inputs = false;               // Setting moving averages once.

    // Dynamixel specific variables
    static int primary_dxl_id = 11;                                 // dynamixel ID number.
    static int baudrate_idx = 3;                                    // 1,000,000 (see drawStatusWindow)
    static const char* devicename = "/dev/ttyUSB?";                 // device name as used by portHandler
    static update_joints_struct ujs = {{0,0,0,0,0}, false};         // updateJoints() requires this return type: joint positions (int32_t) and success: if the message was received sucessfully.
    static std::array<uint8_t, 5> opmodes = {99, 99, 99, 99, 99};   // operating modes for our 5 dynamixels

    // moving average logic: initialize inputs (from user) and outputs (moving average)
    static std::array<int32_t, 5> input_array = {2000, 1000, 3000, 940, 2000}; 
    static std::array<int32_t, 5> output_array; 

    // Create moving average arrays
    static int indexMA = 0;
    static int32_t ma_j0[BUFFER_SIZE];
    static int32_t ma_j1[BUFFER_SIZE];
    static int32_t ma_j2[BUFFER_SIZE];
    static int32_t ma_j3[BUFFER_SIZE];
    static int32_t ma_j4[BUFFER_SIZE];

    // Fill moving average arrays with default values
    static bool is_ma_init = false; 
    if (!is_ma_init) { 
        std::fill(std::begin(ma_j0), std::end(ma_j0), 2000); // for our moving averages. Fill array with an initial value.
        std::fill(std::begin(ma_j1), std::end(ma_j1), 1000);
        std::fill(std::begin(ma_j2), std::end(ma_j2), 3000);
        std::fill(std::begin(ma_j3), std::end(ma_j3), 940);
        std::fill(std::begin(ma_j4), std::end(ma_j4), 2000);
        is_ma_init = true;
    }

    // Index progresses across our array. Keep filling our input arrays and calculating our moving average
    ma_j0[indexMA] = input_array[0];
    ma_j1[indexMA] = input_array[1];
    ma_j2[indexMA] = input_array[2];
    ma_j3[indexMA] = input_array[3];
    ma_j4[indexMA] = input_array[4];
    output_array[0] = calcAvg(ma_j0);
    output_array[1] = calcAvg(ma_j1);
    output_array[2] = calcAvg(ma_j2);
    output_array[3] = calcAvg(ma_j3);
    output_array[4] = calcAvg(ma_j4);

    indexMA = (indexMA + 1) % BUFFER_SIZE;
    
    // gui windows:
    // sample servo controller for debugging
    static int32_t needle1 = 0; 
    static bool knob_is_enabled = false;
    

    // sample visual status box
    drawStatusBoxes(is_connected, is_torqued, is_homed);
    
    // draw arm skeleton
    static std::array<float, 5> floatjointsarray;
    for (int i = 0; i < 5; i++) {
        int32_t measured = ujs.joints_array[i];
        floatjointsarray[i] = ((float)measured * 2 * IM_PI * 0.000244140625);
    }
    drawArmSkeleton(floatjointsarray);

    
    // initialize GroupBulkRead and GroupBulkWrite objects
    // dynamixel::PortHandler* portHandler = nullptr;
    static dynamixel::PortHandler* portHandler = dynamixel::PortHandler::getPortHandler(devicename); 
    static dynamixel::PacketHandler* packetHandler = dynamixel::PacketHandler::getPacketHandler(2.0); 
    static dynamixel::GroupBulkWrite groupBulkWrite(portHandler, packetHandler);
    static dynamixel::GroupBulkRead groupBulkRead(portHandler, packetHandler);

    devicename = drawStatusWindow(primary_dxl_id, baudrate_idx, is_connected, is_homed, is_connection_requested, is_torque_requested, is_home_requested, is_stow_requested, is_disconnect_requested, packetHandler, portHandler, opmodes);

    static float visual_limits[5][2] = {
        {3114.0f, 5091.0f}, 
        {3100.0f, 5640.0f},
        {3175.0f, 5100.0f},
        {3175.0f, 4845.0f},
        {2693.0f, 5075.0f}
    };
    static float literal_limits[5][2] = {
        {90.0f, 270.0f},
        {90.0f, 220.0f},
        {85.0f, 270.0f},
        {85.0f, 290.0f},
        {128.0f, 270.0f}
    };

    drawSettingsWindow(io, enable_vsync, visual_limits, literal_limits);
    // needle1 = showServoWindow("test_knob", needle1, 1000, 1535, true, visual_limits[0][0], visual_limits[0][1], literal_limits[0][0], literal_limits[0][1]); // example window
    input_array[0] = showServoWindow("joint 0 (base)", input_array[0], output_array[0], ujs.joints_array[0], knob_is_enabled, visual_limits[0][0], visual_limits[0][1], literal_limits[0][0], literal_limits[0][1], opmodes[0]);
    input_array[1] = showServoWindow("joint 1", input_array[1], output_array[1], ujs.joints_array[1], knob_is_enabled,        visual_limits[1][0], visual_limits[1][1], literal_limits[1][0], literal_limits[1][1], opmodes[1]);
    input_array[2] = showServoWindow("joint 2", input_array[2], output_array[2], ujs.joints_array[2], knob_is_enabled,        visual_limits[2][0], visual_limits[2][1], literal_limits[2][0], literal_limits[2][1], opmodes[2]);
    input_array[3] = showServoWindow("joint 3", input_array[3], output_array[3], ujs.joints_array[3], knob_is_enabled,        visual_limits[3][0], visual_limits[3][1], literal_limits[3][0], literal_limits[3][1], opmodes[3]);
    input_array[4] = showServoWindow("joint 4", input_array[4], output_array[4], ujs.joints_array[4], knob_is_enabled,        visual_limits[4][0], visual_limits[4][1], literal_limits[4][0], literal_limits[4][1], opmodes[4]);

    // Print out state transition
    std::array<std::string, 8> state_names = {  "STATE_DISCONNECTED",
                                                "STATE_CONNECTING",
                                                "STATE_CONNECTED_ONLY",
                                                "STATE_CONNECTED_TORQUE_ONLY",
                                                "STATE_CONNECTED_TORQUE_HOME",
                                                "STATE_HANDLE_TORQUE_REQUEST",
                                                "STATE_HANDLE_HOME_REQUEST",
                                                "STATE_MOVE_TO_HOME"};

    static int last_state = current_state;
    if (last_state != current_state) {
        printf("\nState Transition\n current_state: %s (%d)\n", state_names[current_state].c_str(), current_state);
    }
    last_state = current_state;


    // State machine
    switch (current_state) {

        case STATE_DISCONNECTED:

            // Arm status
            is_connected = false;
            is_torqued = false;
            is_homed = false;

            // Disable these requests so they stay unraised (looping state)
            is_torque_requested = false;
            is_home_requested = false;
            is_stow_requested = false;
            is_disconnect_requested = false;
            
            // Enable these requests
            if (is_connection_requested) {
                is_connection_requested = false;
                current_state = STATE_CONNECTING;
                break;
            }


            break;

        case STATE_CONNECTING:
        
            if (devicename == "No ports available!") { // return a bool instead of checking the string.
                printf("[connection requested] no ports are available!\n");
                
                current_state = STATE_DISCONNECTED;
                break;

            } else {    
                
                // support a disconnection. using a new port, connect with these new dynamixel objects.
                portHandler = dynamixel::PortHandler::getPortHandler(devicename);
                groupBulkWrite = dynamixel::GroupBulkWrite(portHandler, packetHandler);
                groupBulkRead = dynamixel::GroupBulkRead(portHandler, packetHandler);
                bool is_initialized = initDynamixel(portHandler, groupBulkRead, groupBulkWrite, baudrate_idx, primary_dxl_id);

                if (is_initialized) {                

                    // Check operating mode 
                    opmodes = checkOpMode(packetHandler, portHandler);

                    bool allInPosMode = true;
                    for (int i = 0; i<5; i++) {
                        printf("op mode dxl %d: %d\n", i, opmodes[i]);
                        if (opmodes[i] != 3) {
                            printf("[check op modes] Dynamixel id %d is in operating mode %d instead of 3 (position)! Go into settings to set operating modes.\n", 11 + i, opmodes[i]);
                            allInPosMode = false;
                        }
                    }

                    if (allInPosMode) {
                        printf("[check op modes] Verified all dynamixel operating modes are position!\n");
                        current_state = STATE_CONNECTED_ONLY;
                        is_connected = true;
                        break;
                    } else {
                        current_state = STATE_CONNECTED_ONLY; // this is dumb, apparently memory leak if I try to init dyanmixel within settings window
                        is_connected = true;
                        break;
                    }
                    
                    break;

                } else {
                    current_state = STATE_DISCONNECTED;        
                    is_connected = false;
                    break;
                }
                break;
            }
            
            break;

        case STATE_CONNECTED_ONLY:

            // Disable these requests so they stay unraised (looping state)
            // is_torque_requested = false;
            is_home_requested = false;
            is_stow_requested = false;
            is_disconnect_requested = false;

            // Get new joint positions
            ujs = updateJoints(groupBulkRead, packetHandler, primary_dxl_id);
            if (!ujs.success) {
                current_state = STATE_DISCONNECTED;
                break;
            }

            // Handle a *home* request. Need to change these bool names 
            if (is_torque_requested) {
                is_torque_requested = false;
                current_state = STATE_HANDLE_TORQUE_REQUEST;
                break;
            }

            break;
        
        case STATE_HANDLE_TORQUE_REQUEST:

            // Get new joint positions
            ujs = updateJoints(groupBulkRead, packetHandler, primary_dxl_id);
            if (!ujs.success) {
                current_state = STATE_DISCONNECTED;
                break;
            }

            if (enableTorque(portHandler, packetHandler, primary_dxl_id)) {
                is_torqued = true;
                current_state = STATE_CONNECTED_TORQUE_ONLY;
                break;
            } else {
                is_torqued = false; 
                current_state = STATE_CONNECTED_ONLY;
                break;
            }

            break;
        
        case STATE_CONNECTED_TORQUE_ONLY:   // Runs once. Immediately moves to home request.

            // Get new joint positions
            ujs = updateJoints(groupBulkRead, packetHandler, primary_dxl_id);
            if (!ujs.success) {
                current_state = STATE_DISCONNECTED;
                break;
            }

            is_connected = true;
            is_torqued = true;
            is_homed = false;

            // reset home_pos from disconnected state
            home_pos = {2048, 2048, 2048, 2048, 2048};
            current_state = STATE_HANDLE_HOME_REQUEST;
            break;

        case STATE_HANDLE_HOME_REQUEST:     // Runs once. Calculates quintic coefficients and moves to STATE_MOVE_TO_HOME
            
            // is_connection_requested = false;
            // is_home_requested = false;
            // is_stow_requested = false; // check this. didn't work a second ago???
            // is_torque_requested = false;

            // Set all inputs to current position
            is_ma_inputs = false; 
            if (!is_ma_inputs) { 
                input_array = ujs.joints_array;
                output_array = ujs.joints_array;
                std::fill(std::begin(ma_j0), std::end(ma_j0), ujs.joints_array[0]); // for our moving averages. Fill array with an initial value.
                std::fill(std::begin(ma_j1), std::end(ma_j1), ujs.joints_array[1]);
                std::fill(std::begin(ma_j2), std::end(ma_j2), ujs.joints_array[2]);
                std::fill(std::begin(ma_j3), std::end(ma_j3), ujs.joints_array[3]);
                std::fill(std::begin(ma_j4), std::end(ma_j4), ujs.joints_array[4]);
                is_ma_inputs = true;
            }

            // Calculate quintic spline coeffs for all joints
            time_start = ImGui::GetTime();
            printf("time start calculation: %f\n", time_start);
            time_end = time_start + 5.0; 

            for (int i = 0; i < 5; i++) {
                double start_pos = ujs.joints_array[i];
                std::array<double, 8> boundary_conditions = {0.0f, 5.0f, ujs.joints_array[i], 0, 0, home_pos[i], 0, 0};
                joints_coeffs[i] = {calcCoeff(boundary_conditions)};
                for (int j = 0; j < 6; j++) {
                    printf("%f ",joints_coeffs[i][j]);
                }
                printf("\n");
            }

            
            current_state = STATE_MOVE_TO_HOME;

            break;

        case STATE_MOVE_TO_HOME:            // Loops. Calculates desired position according to quintic spline and commands the arm to move there.

            // Disable these requests so they stay unraised (looping state)
            is_torque_requested = false;
            is_home_requested = false;
            is_stow_requested = false;
            // is_disconnect_requested = false; // I need this to stay raised for "stow and disconnect" functionality

            // Get new joint positions
            ujs = updateJoints(groupBulkRead, packetHandler, primary_dxl_id);
            if (!ujs.success) {
                current_state = STATE_DISCONNECTED;
                break;
            }

            // loops. moves the arm to the home position.
            current_time = ImGui::GetTime();
            std::array<int32_t, 5> quintic_eval;

            for (int i = 0; i < 5; i++) {
                // printf("[is_homing]current_time: %f, time_start: %f, difference: %f\n", current_time, time_start, current_time-time_start);
                std::array<double, 3> pva = calcCurrentPVA(joints_coeffs[i], current_time - time_start);
                int32_t desired = (int32_t)pva[0];
                quintic_eval[i] = desired;
            }
            
            if (!setJoints(ujs.joints_array, quintic_eval, groupBulkWrite, packetHandler, primary_dxl_id)) {
                printf("something wrong at set_joints success?\n");
                knob_is_enabled = false;
                current_state = STATE_CONNECTED_TORQUE_ONLY;
                break;
            }

            if (ImGui::GetTime() > time_end) { // if we're at the end of the homing procedure, set inputs to current pos

                // fill inputs with where we *should* to be
                input_array = home_pos;
                output_array = home_pos;
                std::fill(std::begin(ma_j0), std::end(ma_j0), home_pos[0]); // for our moving averages. Fill array with an initial value.
                std::fill(std::begin(ma_j1), std::end(ma_j1), home_pos[1]);
                std::fill(std::begin(ma_j2), std::end(ma_j2), home_pos[2]);
                std::fill(std::begin(ma_j3), std::end(ma_j3), home_pos[3]);
                std::fill(std::begin(ma_j4), std::end(ma_j4), home_pos[4]);

                is_homing = false; 
                is_homed = true;
                is_home_requested = false;
                current_state = STATE_CONNECTED_TORQUE_HOME;
                is_coeff_calcd = false;
                break;
            }

            break;

        case STATE_CONNECTED_TORQUE_HOME:

            // Disable these requests so they stay unraised (looping state)
            // is_torque_requested = false;
            is_home_requested = false;
            // is_stow_requested = false;
            // is_disconnect_requested = false;

            // Get new joint positions
            ujs = updateJoints(groupBulkRead, packetHandler, primary_dxl_id);
            if (!ujs.success) {
                current_state = STATE_DISCONNECTED;
                break;
            }

            if (is_torque_requested) {
                is_torque_requested = false;

                home_pos = {2048, 2048, 2048, 2048, 2048};
                
                // Check if already at home position
                if (diffWithinLimits(ujs.joints_array, home_pos, -10, 10)) {
                    printf("Already at home position!\n");
                } else {
                    current_state = STATE_HANDLE_HOME_REQUEST;
                    break;
                }                

            }

            if (is_stow_requested) {
                is_stow_requested = false;
                home_pos = {3084, 984, 2992, 974, 2046};

                for (int i=0;i<5;i++) {
                    printf("ujs.joints_array: %d, ", ujs.joints_array[i]);
                    printf("home_pos: %d\n", home_pos[i]);
                }

                if (diffWithinLimits(ujs.joints_array, home_pos, -40, 40)) {
                    printf("Already at stowed position!\n");
                } else {
                    current_state = STATE_HANDLE_HOME_REQUEST;
                    break;
                }
                
            }

            if (is_disconnect_requested) {
                std::array<int32_t, 5> stowed_pos = {3084, 995, 2992, 979, 2046};

                if (diffWithinLimits(ujs.joints_array, stowed_pos, -40, 40)) {
                    disableTorque(portHandler, packetHandler, 11);

                    is_disconnect_requested = false;

                    current_state = STATE_DISCONNECTED;
                    break;
                } else {
                    is_stow_requested = true;
                }
                
            }

            // set joint positions
            if (!setJoints(ujs.joints_array, output_array, groupBulkWrite, packetHandler, primary_dxl_id)) {
                printf("something wrong at set_joints success?\n");
                knob_is_enabled = false;
                current_state = STATE_CONNECTED_TORQUE_ONLY;
                break;
            }


            knob_is_enabled = true;           

            break;
            
        default:
            // Code for unexpected values
            printf("Unexpected state value: %d\n", current_state);
            break;
    }

}