#version 130

attribute vec3 position;
attribute vec2 texCoord;
attribute vec3 normal;
attribute vec3 colorCoord;

varying vec2 texCoord0;
varying vec3 normal0;
varying vec3 position0;
varying vec3 colorCoord0;
varying vec3 camPos;

uniform mat4 MVP;
uniform vec3 cameraPos;
uniform vec4 clipingPlane;

void main()
{
	gl_Position = MVP * vec4(position, 1.0);
	position0 =  position;
	texCoord0 = texCoord;
	normal0 = (vec4(normal, 0.0)).xyz;
	camPos = cameraPos;
	vec4 position_w = vec4(position, 1.0);
   	gl_ClipDistance[0] = 1;
	gl_ClipDistance[1] = 1;
   	gl_ClipDistance[2] = 1;
   	gl_ClipDistance[3] = 1;
   	gl_ClipDistance[4] = 1;


}