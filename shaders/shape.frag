#version 460

layout(set = 2, binding = 0) uniform UniformBufferObject3 {
    mat4 mat;
} color_mat;

layout(location = 0) in vec3 fragUv;
layout(location = 1) in vec4 fragColor;
layout(location = 2) in flat uint inFlags;


layout(location = 0) out vec4 outColor;

float aa_quadratic(vec3 uv) {
    float q = uv.z * (uv.x * uv.x - uv.y);

    vec2 dx = dFdx(uv.xy);
    vec2 dy = dFdy(uv.xy);

    float gx = uv.z * (2.0 * uv.x * dx.x - dx.y);
    float gy = uv.z * (2.0 * uv.x * dy.x - dy.y);

    float denom = max(sqrt(gx * gx + gy * gy), 1e-6);
    float sd = q / denom;
    return clamp(0.5 + sd, 0.0, 1.0);
}

float aa_cubic(vec3 uv) {
    float c = uv.x * uv.x * uv.x - uv.y * uv.z;

    vec3 dx = dFdx(uv);
    vec3 dy = dFdy(uv);

    float gx = 3.0 * uv.x * uv.x * dx.x - dx.y * uv.z - uv.y * dx.z;
    float gy = 3.0 * uv.x * uv.x * dy.x - dy.y * uv.z - uv.y * dy.z;

    float denom = max(sqrt(gx * gx + gy * gy), 1e-6);
    float sd = c / denom;
    return clamp(0.5 + sd, 0.0, 1.0);
}

void main() {
    float alpha = 1.0;

    if (inFlags == 2) { //not curve
    } else if (inFlags == 1) { //not cubic curve
        alpha = aa_quadratic(fragUv);
    } else {
        alpha = aa_cubic(fragUv);
    }

    if (alpha <= 0.0) discard;

    outColor = color_mat.mat * fragColor;
    outColor.a *= alpha;
}
