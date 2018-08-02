#version 130

attribute vec3 position;
attribute vec2 texCoord;
attribute vec3 colorCoord;
attribute vec3 normal;

varying vec2 texCoord0;
varying vec3 position0;
varying vec3 colorCoord0;
varying vec3 camPos;
varying vec3 normal0;

uniform mat4 MVP;
uniform mat4 Normal;
uniform vec3 cameraPos;
uniform vec3 colorMono;
uniform vec4 clipingPlane0;
uniform vec4 clipingPlane1;
uniform vec4 clipingPlane2;
uniform vec4 clipingPlane3;
uniform vec4 clipingPlane4;

void main()
{
	gl_Position = MVP * vec4(position, 1.0);
	position0 =  (Normal* vec4(position, 1.0)).xyz;
	texCoord0 = texCoord;
	colorCoord0 = colorMono;
	camPos = cameraPos;
	normal0 = normal;
    vec4 position_w = Normal * vec4(position, 1.0);
   	gl_ClipDistance[0] = dot( clipingPlane0, position_w);
  	gl_ClipDistance[1] = dot( clipingPlane1, position_w);
  	gl_ClipDistance[2] = dot( clipingPlane2, position_w);
    gl_ClipDistance[3] = dot( clipingPlane3, position_w);
    gl_ClipDistance[4] = dot( clipingPlane4, position_w);
}