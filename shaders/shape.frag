#version 460

#define USE_FAST_CURVE_AA 1

layout(set = 2, binding = 0, std140) uniform UniformBufferObject3 {
    mat4 mat;
    vec4 offset;
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

    float g2 = max(gx * gx + gy * gy, 1e-12);

    return clamp(0.5 + q * inversesqrt(g2), 0.0, 1.0);
}

float aa_cubic(vec3 uv) {
    float x2 = uv.x * uv.x;
    float c = x2 * uv.x - uv.y * uv.z;

    vec3 dx = dFdx(uv);
    vec3 dy = dFdy(uv);

    float gx = 3.0 * x2 * dx.x - dx.y * uv.z - uv.y * dx.z;
    float gy = 3.0 * x2 * dy.x - dy.y * uv.z - uv.y * dy.z;

    float g2 = max(gx * gx + gy * gy, 1e-12);

    return clamp(0.5 + c * inversesqrt(g2), 0.0, 1.0);
}


float aa_quadratic_fast(vec3 uv) {
    float q = uv.z * (uv.x * uv.x - uv.y);
    float w = max(fwidth(q), 1e-6);
    return clamp(0.5 + q / w, 0.0, 1.0);
}

float aa_cubic_fast(vec3 uv) {
    float c = uv.x * uv.x * uv.x - uv.y * uv.z;
    float w = max(fwidth(c), 1e-6);
    return clamp(0.5 + c / w, 0.0, 1.0);
}

void main() {
    float alpha = 1.0;

    if (inFlags == 1u) {
        #if USE_FAST_CURVE_AA
            alpha = aa_cubic_fast(fragUv);
        #else
            alpha = aa_cubic(fragUv);
        #endif
    } else if (inFlags == 2u) {
        #if USE_FAST_CURVE_AA
            alpha = aa_quadratic_fast(fragUv);
        #else
            alpha = aa_quadratic(fragUv);
        #endif
    }


    outColor = color_mat.mat * fragColor + color_mat.offset;
    outColor.a *= alpha;
}