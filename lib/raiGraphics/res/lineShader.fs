#version 130
uniform vec3 colorMono;

void main()
{
        gl_FragColor=vec4(colorMono,1);  //<- don't forget the paranthesis (ambient+diffuse+specular)
}