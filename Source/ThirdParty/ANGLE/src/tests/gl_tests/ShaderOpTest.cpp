//
// Copyright 2024 The ANGLE Project Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.
//
// Tests for shader interpolation qualifiers
//

#include "test_utils/ANGLETest.h"
#include "test_utils/gl_raii.h"

using namespace angle;

constexpr int kPixelColorThreshhold = 8;

class ShaderOpTest : public ANGLETest<>
{
  protected:
    ShaderOpTest() : ANGLETest()
    {
        setWindowWidth(256);
        setWindowHeight(256);
        setConfigRedBits(8);
        setConfigGreenBits(8);
        setConfigBlueBits(8);
        setConfigAlphaBits(8);
        setConfigDepthBits(24);
        setConfigStencilBits(8);
        setMultisampleEnabled(0);
    }
};

// Simplified test from dEQP-GLES2.functional.fragment_ops.interaction.basic_shader.22
// Test that vulkan drivers correctly handle the constant expression spirv generated by ANGLE
TEST_P(ShaderOpTest, ConstantExpression)
{
    const char *vertSrc = R"(
attribute vec4 pos;

void main()
{
    gl_Position = pos;
}
)";
    // ANGLE and glslang translate below expression in fragment shader differently
    // vec4 c = vec4(1.0, 0.5, 0.5, 0.75)

    // ANGLE:
    // %constVec4 = OpConstantComposite %vec4 %const1 %const2 %const2 %const3
    // %var = OpVariable %PrivateVec4Pointer Private %constVec4
    // %loadedVal = OpLoad %vec4 %var
    // OpStore %gl_fragColor %loadedVal

    // glslang:
    // %constVec4 = OpConstantComposite %vec4 %const1 %const2 %const2 %const3
    // %var = OpVariable %PrivateVec4Pointer Private
    // OpStore %var %constVec4
    // %loadedVal = OpLoad %vec4 %var
    // OpStore %gl_fragColor %loadedVal

    // Both are valid spirv instructions.
    // Tests should pass with spirv generated by ANGLE.

    // Note: setting forceDeferNonConstGlobalInitializers to true will make ANGLE generate the same
    // spirv instruction for the constant expression as glslang.
    const char *fragSrc = R"(
precision mediump float;
vec4 c = vec4(1.0, 0.5, 0.5, 0.75);

void main()
{
    gl_FragColor = c;
}
)";
    ANGLE_GL_PROGRAM(program, vertSrc, fragSrc);
    glUseProgram(program);

    std::array<GLfloat, 16> attribPosData = {1, 1,  0.5, 1, -1, 1,  0.5, 1,
                                             1, -1, 0.5, 1, -1, -1, 0.5, 1};
    GLint attribPosLoc                    = glGetAttribLocation(1, "pos");
    ASSERT(attribPosLoc >= 0);
    glEnableVertexAttribArray(attribPosLoc);
    glVertexAttribPointer(attribPosLoc, 4, GL_FLOAT, GL_FALSE, 0, attribPosData.data());

    const uint16_t indices[] = {0, 1, 2, 2, 1, 3};
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_SHORT, &indices[0]);
    EXPECT_PIXEL_COLOR_NEAR(64, 64, GLColor(255, 125, 125, 190), kPixelColorThreshhold);
}

// Simplified test from dEQP-GLES2.functional.shaders.random.all_features.fragment.12
// Test that vulkan drivers correctly handle the constant expression spirv generated by ANGLE
// Difference beween this test and ConstantExpression test is there is a uniform in fragment shader
// in this test, and with this extra element the ANGLE spirv can trigger an error in
// vkCreateGraphicsPipelines on some drivers.
TEST_P(ShaderOpTest, ConstantExpressionComparedWithUniform)
{
    const char *vertSrc = R"(
attribute vec4 pos;
void main()
{
    gl_Position = pos;
}
)";

    // ANGLE and glslang translate below expression in fragment shader differently
    // int k = (5) * (1)

    // ANGLE:
    // %k = OpVariable %_ptr_Private_int_0 Private %int_5 // declare Private OpVariable with
    // initializer
    // ... some other instructions
    // %main = OpFunction %void None %62
    // %67 = OpLoad %int %k
    // ... some other instructions

    // glslang:
    // %k = OpVariable %_ptr_Private_int_0 Private // declare Private OpVariable without initializer
    // ... some other instructions
    // %main = OpFunction %void None %62
    // OpStore %k %int_5 // Assign the value to the Private OpVariable in the main function
    // %67 = OpLoad %int %k
    // ... some other instructions

    // Both are valid spirv instructions.
    // Tests should pass with spirv generated by ANGLE

    // Note: setting forceDeferNonConstGlobalInitializers to true will make ANGLE generate the same
    // spirv instruction for the constant expression as glslang.

    const char *fragSrc = R"(
precision mediump float;
int k = (5) * (1);
uniform mediump int o;
void main()
{
    gl_FragColor = vec4(1.0, 1.0, 1.0, 1.0);
    if (o >= k)
        gl_FragColor = vec4(0.0, 0.0, 1.0, 1.0);
}
)";
    ANGLE_GL_PROGRAM(program, vertSrc, fragSrc);
    glUseProgram(program);
    EXPECT_GL_NO_ERROR();

    std::array<GLfloat, 16> attribPosData = {1, 1,  0.5, 1, -1, 1,  0.5, 1,
                                             1, -1, 0.5, 1, -1, -1, 0.5, 1};
    GLint attribPosLoc                    = glGetAttribLocation(1, "pos");
    ASSERT(attribPosLoc >= 0);
    glEnableVertexAttribArray(attribPosLoc);
    glVertexAttribPointer(attribPosLoc, 4, GL_FLOAT, GL_FALSE, 0, attribPosData.data());
    EXPECT_GL_NO_ERROR();

    GLint uniformOLocation = glGetUniformLocation(program, "o");
    ASSERT(uniformOLocation >= 0);
    GLint uniformOValue = 4;
    glUniform1i(uniformOLocation, uniformOValue);

    const uint16_t indices[] = {0, 1, 2, 2, 1, 3};
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_SHORT, &indices[0]);
    EXPECT_GL_NO_ERROR();
    EXPECT_PIXEL_COLOR_NEAR(64, 64, GLColor(255, 255, 255, 255), kPixelColorThreshhold);
}

GTEST_ALLOW_UNINSTANTIATED_PARAMETERIZED_TEST(ShaderOpTest);
ANGLE_INSTANTIATE_TEST_ES2(ShaderOpTest);
