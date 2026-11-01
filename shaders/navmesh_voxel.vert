#version 430 core
layout(location = 0) in vec3 position;
layout(location = 1) in vec3 voxelMinimum;
layout(location = 2) in vec3 voxelSize;
layout(location = 3) in vec3 voxelColor;
uniform mat4 view;
uniform mat4 projection;
out vec3 instanceColor;
flat out vec3 faceNormal;

// 共有立方体の頂点は、外向きの面ごとに三角形２枚ずつ並んでいる。
const vec3 faceNormals[6] = vec3[6](
    vec3(0.0, 0.0, -1.0), vec3(0.0, 0.0, 1.0),
    vec3(-1.0, 0.0, 0.0), vec3(1.0, 0.0, 0.0),
    vec3(0.0, -1.0, 0.0), vec3(0.0, 1.0, 0.0));

void main()
{
    gl_Position = projection * view * vec4(voxelMinimum + position * voxelSize, 1.0);
    instanceColor = voxelColor;
    faceNormal = faceNormals[gl_VertexID / 6];
}
