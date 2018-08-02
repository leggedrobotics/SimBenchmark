#version 130
in vec3 position;
out vec3 TexCoords;

uniform mat4 MVP;


void main()
{
    gl_Position =   MVP * vec4(position, 1.0);
    TexCoords = position;
    gl_ClipDistance[0] = 1;
    gl_ClipDistance[1] = 1;
    gl_ClipDistance[2] = 1;
    gl_ClipDistance[3] = 1;
    gl_ClipDistance[4] = 1;
}