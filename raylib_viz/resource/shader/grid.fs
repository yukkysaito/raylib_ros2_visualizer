#version 330 core
out vec4 FragColor;

void main()
{   
    FragColor = vec4(0.8, 0.8, 0.8, 1.0);
    gl_FragDepth = 1.0;
}
