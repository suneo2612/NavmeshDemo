#version 430 core
uniform vec3 lineColor;
out vec4 color;
void main()
{
    color = vec4(lineColor, 1.0);
}
