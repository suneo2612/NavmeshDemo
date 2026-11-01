#version 430 core
in vec3 instanceColor;
flat in vec3 faceNormal;
out vec4 color;

// 影側も視認できる環境光と、上方からの方向光を組み合わせる。
const float ambientLight = 0.3;
const float directionalLight = 0.7;
const vec3 lightDirection = vec3(0.4, 1.0, 0.6);

void main()
{
    float diffuse = max(dot(faceNormal, normalize(lightDirection)), 0.0);
    color = vec4(instanceColor * (ambientLight + directionalLight * diffuse), 1.0);
}
