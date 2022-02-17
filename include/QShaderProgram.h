//
// Created by jun on 16/02/2022.
//

#ifndef QSHADERPROGRAM_H
#define QSHADERPROGRAM_H

#include <GL/glew.h>
#include <string>
#include <iostream>
#include <fstream>

namespace QShaderProgram
{
    extern unsigned int shader;

    std::string readShaderFile(const char*);
    unsigned int shaderCompile(unsigned int, const std::string& );
    unsigned int createShaders(const std::string&, const std::string&);
}


#endif // QSHADERPROGRAM_H
