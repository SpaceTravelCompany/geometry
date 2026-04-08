#version 460

layout(set = 2, binding = 0) uniform UniformBufferObject3 {
    mat4 mat;
} color_mat;

layout(location = 0) in vec3 fragUv;
layout(location = 1) in vec4 fragColor;
layout(location = 2) in flat uint inFlags;


layout(location = 0) out vec4 outColor;

// float aa_quadratic(vec3 uv) {
//     float q = uv.z * (uv.x * uv.x - uv.y);

//     vec2 dx = dFdx(uv.xy);
//     vec2 dy = dFdy(uv.xy);

//     float gx = uv.z * (2.0 * uv.x * dx.x - dx.y);
//     float gy = uv.z * (2.0 * uv.x * dy.x - dy.y);

//     float sd = q / sqrt(gx * gx + gy * gy);
//     return clamp(0.5 + sd, 0.0, 1.0);
// }

// float aa_cubic(vec3 uv) {
//     float c = uv.x * uv.x * uv.x - uv.y * uv.z;

//     vec3 dx = dFdx(uv);
//     vec3 dy = dFdy(uv);

//     float gx = 3.0 * uv.x * uv.x * dx.x - dx.y * uv.z - uv.y * dx.z;
//     float gy = 3.0 * uv.x * uv.x * dy.x - dy.y * uv.z - uv.y * dy.z;

//     float sd = c / sqrt(gx * gx + gy * gy);
//     return clamp(0.5 + sd, 0.0, 1.0);
// }
float aa_quadratic(vec3 uv) {
    float q = uv.z * (uv.x * uv.x - uv.y);
    return clamp(0.5 + q / fwidth(q), 0.0, 1.0);
}

float aa_cubic(vec3 uv) {
    float c = uv.x * uv.x * uv.x - uv.y * uv.z;
    return clamp(0.5 + c / fwidth(c), 0.0, 1.0);
}

float aa_linear(float d) {
    return clamp(0.5 + d / fwidth(d), 0.0, 1.0);
}

void main() {
    float alpha = 1.0;

   if (inFlags == 0) {
        alpha = aa_cubic(fragUv);
    } else if (inFlags == 1) {
        alpha = aa_quadratic(fragUv);
    } else if (inFlags == 3) { // LINE_EDGE
        alpha = aa_linear(fragUv.x);
    } else {
        alpha = 1.0; // interior fill triangle
    }

    if (alpha <= 0.0) discard;

    outColor = color_mat.mat * fragColor;
    outColor.a *= alpha;
}