#version 130
varying vec3 position0;
varying vec3 normal0;
varying vec3 colorCoord0;
varying vec3 camPos;

uniform vec3 colorMono;
uniform vec3 lightPos;

uniform vec3 mambient;  //gl_FrontMaterial
uniform vec3 mdiffuse;
uniform vec3 mspecular;
uniform float shininess;

uniform vec3 lambient;  //gl_LightSource[0]
uniform vec3 ldiffuse;
uniform vec3 lspecular;
uniform float transparency;
uniform vec4 clipingPlane;
uniform bool hasColorCoord;

void main()
{
        float dist=length(position0-lightPos);   //distance from light-source to surface
        float att=1.0;    //attenuation (constant,linear,quadric)
        vec3 ambient=mambient*lambient; //the ambient light

        vec3 surf2light=normalize(lightPos-position0);
        vec3 norm=normalize(normal0);
        float dcont=max(0.0,dot(norm,surf2light));
        vec3 diffuse=dcont*(mdiffuse*ldiffuse);

        vec3 surf2view=normalize(position0-camPos);
        vec3 reflection=reflect(surf2light,norm);

        float scont=pow(max(0.0,dot(surf2view,reflection)),shininess);
        vec3 specular=scont*lspecular*mspecular;
        gl_FragColor=vec4(colorMono * (ambient+diffuse+specular)*att,transparency);  //<- don't forget the paranthesis (ambient+diffuse+specular)


}