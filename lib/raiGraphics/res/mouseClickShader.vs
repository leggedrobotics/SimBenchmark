#version 130
uniform mat4 MVP;
attribute vec3 position;

void main()
{
	gl_Position = MVP * vec4(position, 1.0);
    gl_ClipDistance[0] = 1;
    gl_ClipDistance[1] = 1;
    gl_ClipDistance[2] = 1;
    gl_ClipDistance[3] = 1;
    gl_ClipDistance[4] = 1;

}