#version 130

uniform sampler2D tex;

in vec2 fragTexCoord;

out vec4 finalColor;
in float trans;

void main() {
    finalColor = texture(tex, fragTexCoord);

    if(finalColor.w==0) {
        finalColor.x = 0;
        finalColor.y = 0;
        finalColor.z = 0;
        finalColor.w = trans;
        }
}