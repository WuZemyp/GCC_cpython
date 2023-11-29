#include "C:\Users\13513\ALVR_Private\ALVR\glm\glm\glm.hpp"
#include "C:\Users\13513\ALVR_Private\ALVR\glm\glm\gtc\quaternion.hpp"
#include "C:\Users\13513\ALVR_Private\ALVR\glm\glm\gtc\matrix_transform.hpp"
#include <cmath>
#include <math.h>
#include <iostream>
const double PI = 3.14159265358979323846;
double radiansToDegrees(double radians) {
    return radians * 180.0 / PI;
}
int main(){
    glm::mat4 viewMatrix = glm::translate(glm::mat4(1.0f), glm::vec3(0.20107618, 1.2200207, -0.06837921)) *
                       glm::mat4_cast(glm::quat(-0.24396048, -0.35554647, 0.15731739, -0.8884375));
    float nearClip = 0.1f; // Near clipping plane distance
    float farClip = 1000.0f; // Far clipping plane distance
    glm::mat4 projectionMatrix = glm::perspectiveRH_ZO(glm::radians(radiansToDegrees(0.6981317) - radiansToDegrees(-0.94247776)),
                                                   glm::radians(radiansToDegrees(0.7330383) - radiansToDegrees(-0.94247776)),
                                                   static_cast<double>(nearClip), static_cast<double>(farClip));
    glm::mat4 modelMatrix = glm::translate(glm::mat4(1.0f), glm::vec3(-0.16990215, -0.13591814, -0.50386375)) *
                       glm::mat4_cast(glm::quat(-0.14068037, -0.14248136, -0.001580268, 0.9797477));
    glm::vec4 objectCoordinates(0.5f, 0.5f, 0.5f, 1.0f);
    glm::mat4 mvpMatrix = projectionMatrix * viewMatrix * modelMatrix;
    glm::vec4 clipSpaceCoordinates = mvpMatrix * objectCoordinates;
    glm::vec3 ndcCoordinates = glm::vec3(
    clipSpaceCoordinates.x / clipSpaceCoordinates.w,
    clipSpaceCoordinates.y / clipSpaceCoordinates.w,
    clipSpaceCoordinates.z / clipSpaceCoordinates.w
    );

    // Convert NDC coordinates to screen coordinates
    float screenWidth = 2144.0f;
    float screenHeight = 1072.0f;
    glm::vec2 screenCoordinates = glm::vec2(
        (ndcCoordinates.x + 1.0f) * 0.5f * screenWidth,
        (ndcCoordinates.y + 1.0f) * 0.5f * screenHeight
    );
    std::cout<<"x: "<<screenCoordinates.x<<"y: "<<screenCoordinates.y<<std::endl;
}