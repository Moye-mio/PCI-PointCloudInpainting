#include "pch.h"
#include "Shader.h"

using SPoint = Eigen::Vector3f;
using SColor = Eigen::Vector3f;

class TestBSplineCurve : public testing::Test
{
protected:
	void SetUp() override
	{
	}

	void TearDown() override
	{
	}

	void ProcessInput(GLFWwindow* vWindow)
	{
		if (glfwGetKey(vWindow, GLFW_KEY_ESCAPE) == GLFW_PRESS)
			glfwSetWindowShouldClose(vWindow, true);
	}

	static void FrameBufferSizeCallback(GLFWwindow* vWindow, int vWidth, int vHeight)
	{
		glViewport(0, 0, vWidth, vHeight);
	}

	void generateControlPoints(std::vector<SPoint>& voVertices, int vRows /* x */, int vCols /* y */, int vDepths /* z */)
	{
		for (int i = 0; i < vRows; i++)
			for (int k = vCols - 1; k >= 0; k--)
				for (int m = 0; m < vDepths; m++)
					if (i == 0 || k == 0)
					{
						SPoint Point;
						Point.x() = i * 0.1;
						Point.y() = k * 0.1;
						Point.z() = m * 0.1;
						voVertices.emplace_back(Point);
					}

		for (int i = 0; i < voVertices.size(); i++)
		{
			voVertices[i].x() = voVertices[i].x() * 2 - 0.5f;
			voVertices[i].y() = voVertices[i].y() * 2 - 0.5f;
			voVertices[i].z() = voVertices[i].z() * 2;
		}

		std::cout << "Control Points: " << voVertices.size() << std::endl;
	}

	bool initGL(GLFWwindow* vWindow)
	{
		if (vWindow == NULL)
		{
			std::cout << "Failed to create GLFW window" << std::endl;
			glfwTerminate();
			return -1;
		}
		glfwMakeContextCurrent(vWindow);
		glfwSetFramebufferSizeCallback(vWindow, FrameBufferSizeCallback);

		if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
		{
			std::cout << "Failed to initialize GLAD" << std::endl;
			return -1;
		}

		return 1;
	}

	const unsigned int G_SCRWIDTH = 800;
	const unsigned int G_SCRHEIGHT = 600;
};

TEST_F(TestBSplineCurve, DT_InvalidDegree)
{
	int Degree = -1;
	ASSERT_DEATH(core::CBSplineCurve Curve(Degree), "");
}

TEST_F(TestBSplineCurve, DT_InvalidControlPoints)
{
	int Degree = 3;
	core::CBSplineCurve Curve(Degree);
	ASSERT_DEATH(Curve.setControlPoints(std::vector<Eigen::Vector3f>()), "");
}

TEST_F(TestBSplineCurve, NT_Straight)
{
	std::vector<SPoint> ControlPoints;
	for (int i = 0; i < 4; i++)
		ControlPoints.emplace_back(SPoint(i, i, i));

	int Degree = 3;
	core::CBSplineCurve Curve(Degree);
	Curve.setControlPoints(ControlPoints);
	ASSERT_EQ(Curve.sample(0), Eigen::Vector3f(0, 0, 0));
	ASSERT_EQ(Curve.sample(0.5), Eigen::Vector3f(1.5, 1.5, 1.5));
	ASSERT_EQ(Curve.sample(1), Eigen::Vector3f(3, 3, 3));
}

TEST_F(TestBSplineCurve, NT_DrawCurve)
{
	int Rows = 5;
	int Cols = 5;
	int Depths = 5;
	int Offset = 6;
	int Degree = 3;
	int Scale = 9;

	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	GLFWwindow* pWindow = glfwCreateWindow(G_SCRWIDTH, G_SCRHEIGHT, "BSplineSurface", NULL, NULL);
	ASSERT_TRUE(initGL(pWindow) != -1);

	glEnable(GL_DEPTH_TEST);

	Shader Shader1("shader.vs", "shader.fs");

	std::vector<SPoint> ControlPoints, Nodes;
	SColor ControlPointColor(246.0f / 255.0f, 162.0f / 255.0f, 152.0f / 255.0f);
	SColor NodeColor(168.0f / 255.0f, 160.0f / 255.0f, 207.0f / 255.0f);;
	generateControlPoints(ControlPoints, Rows, Cols, Depths);

	{
		std::cout << "Control Points:" << std::endl;
		for (int i = 0; i < ControlPoints.size(); i++)
			std::cout << ControlPoints[i].x() << ", " << ControlPoints[i].y() << ", " << ControlPoints[i].z() << std::endl;
		std::cout << std::endl << std::endl << "Nodes:" << std::endl;
	}

	core::CBSplineCurve Curve(Degree);
	Curve.setControlPoints(ControlPoints);
	int NodeNumber = (ControlPoints.size() - Degree) * Scale;
	std::vector<float> Paras;
	for (int i = 0; i <= NodeNumber; i++)
	{
		SPoint r = Curve.sample((float)i / (float)NodeNumber);
		Nodes.emplace_back(std::move(r));
	}

	float* ControlPoints4Draw = new float[ControlPoints.size() * (ControlPoints[0].size() + ControlPointColor.size())];
	float* Nodes4Draw = new float[Nodes.size() * (Nodes[0].size() + Nodes.size())];

	int ControlPointsCount = -1;
	for (auto& e : ControlPoints)
	{
		for (int i = 0; i < e.size(); i++)
			ControlPoints4Draw[++ControlPointsCount] = e[i];
		for (int i = 0; i < ControlPointColor.size(); i++)
			ControlPoints4Draw[++ControlPointsCount] = ControlPointColor[i];
	}
	std::cout << "ControlPoints Size:" << ControlPointsCount << std::endl;

	int NodesCount = -1;
	for (auto& e : Nodes)
	{
		for (int i = 0; i < e.size(); i++)
			Nodes4Draw[++NodesCount] = e[i];
		for (int i = 0; i < NodeColor.size(); i++)
			Nodes4Draw[++NodesCount] = NodeColor[i];
	}
	std::cout << "Nodes Size:" << NodesCount << std::endl;

	unsigned int VBO, VAO;
	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);
	glBindVertexArray(VAO);

	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, (ControlPointsCount + 1) * sizeof(float), ControlPoints4Draw, GL_STATIC_DRAW);

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
	glEnableVertexAttribArray(1);

	unsigned int VBO2, VAO2;
	glGenVertexArrays(1, &VAO2);
	glGenBuffers(1, &VBO2);
	glBindVertexArray(VAO2);

	glBindBuffer(GL_ARRAY_BUFFER, VBO2);
	glBufferData(GL_ARRAY_BUFFER, (NodesCount + 1) * sizeof(float), Nodes4Draw, GL_STATIC_DRAW);

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
	glEnableVertexAttribArray(1);

	while (!glfwWindowShouldClose(pWindow))
	{
		ProcessInput(pWindow);

		glClearColor(0.9f, 0.9f, 0.9f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		Shader1.use();

		glm::mat4 model = glm::mat4(1.0f);
		glm::mat4 view = glm::mat4(1.0f);
		glm::mat4 projection = glm::mat4(1.0f);
		projection = glm::perspective(glm::radians(45.0f), (float)G_SCRWIDTH / (float)G_SCRHEIGHT, 0.1f, 100.0f);
		view = glm::translate(view, glm::vec3(0.0f, 0.0f, -3.0f));
		Shader1.setMat4("projection", projection);
		Shader1.setMat4("view", view);
		Shader1.setMat4("model", model);

		glBindVertexArray(VAO);
		glPointSize(10.0f);
		glDrawArrays(GL_POINTS, 0, ControlPoints.size());

		glBindVertexArray(VAO2);
		glLineWidth(1.5f);
		glDrawArrays(GL_LINE_STRIP, 0, Nodes.size());


		glfwSwapBuffers(pWindow);
		glfwPollEvents();
	}

	glDeleteVertexArrays(1, &VAO);
	glDeleteBuffers(1, &VBO);

	glfwTerminate();
}