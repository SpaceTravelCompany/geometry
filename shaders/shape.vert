#version 460

layout(set = 0, binding = 0) uniform UniformBufferObject0 {
    mat4 model;
} model;
layout(set = 1, binding = 0) uniform UniformBufferObject1 {
    mat4 view;
} view;
layout(set = 1, binding = 1) uniform UniformBufferObject2 {
    mat4 proj;
} proj;

layout(location = 0) in vec2 inPosition;
layout(location = 1) in vec3 inUv;
layout(location = 2) in vec4 inColor;
layout(location = 3) in uint inFlags;

layout(location = 0) out vec3 fragUv;
layout(location = 1) out vec4 fragColor;
layout(location = 2) out flat uint outFlags;

void main() {
    gl_Position = proj.proj * view.view * model.model * vec4(inPosition, 0.0, 1.0);
    fragUv = inUv;
    fragColor = inColor;
    outFlags = inFlags;
}
