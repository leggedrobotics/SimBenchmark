#version 130

in vec3 vert;
in vec2 vertTexCoord;
uniform float xscale;
uniform float yscale;
uniform float xpos;
uniform float ypos;
uniform float transparency;

out vec2 fragTexCoord;
out float trans;

void main() {
    // Pass the tex coord straight through to the fragment shader
    fragTexCoord = vertTexCoord;
    trans = transparency;
    gl_Position = vec4(vert.x * xscale + xpos -1, vert.y * yscale + ypos -1, 0, 1);
    //gl_Position = tempVec;
    gl_ClipDistance[0] = 1;
    gl_ClipDistance[1] = 1;
    gl_ClipDistance[2] = 1;
    gl_ClipDistance[3] = 1;
    gl_ClipDistance[4] = 1;
}