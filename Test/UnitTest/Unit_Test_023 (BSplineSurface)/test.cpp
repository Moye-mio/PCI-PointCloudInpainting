#include "pch.h"
#include "Shader.h"

using SColor = Eigen::Vector3f;

class TestBSplineSurface : public testing::Test
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

	void generatePoints(Eigen::Matrix<core::SPoint, -1, -1>& voVertices, int vRows /* x */, int vCols /* y */, int vDepths /* z */)
	{
		voVertices.resize(vRows + vCols - 1, vDepths);

		int Count = -1;
		for (int i = 0; i < vRows; i++)
			for (int k = vCols - 1; k >= 0; k--)
				for (int m = 0; m < vDepths; m++)
					if (i == 0 || k == 0)
					{
						Count++;
						core::SPoint Point;
						Point.x() = i * 0.2 - 0.5f;
						Point.y() = k * 0.2 - 0.5f;
						Point.z() = m * 0.2;
						/*Point.r = 246.0f / 255.0f;
						Point.g = 162.0f / 255.0f;
						Point.b = 152.0f / 255.0f;*/
						voVertices.coeffRef(Count / vDepths, m) = Point;
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

	float calcError(const Eigen::Vector3f& vLhs, const Eigen::Vector3f& vRhs)
	{
		float e = 0.0f;
		for (int i = 0; i < 3; i++)
			e += std::powf(vLhs[i] - vRhs[i], 2);
		return e;
	}

	const unsigned int G_SCRWIDTH = 800;
	const unsigned int G_SCRHEIGHT = 600;
	const float m_Epsilon = 0.00001;
};

TEST_F(TestBSplineSurface, DT_InvalidDegree)
{
	ASSERT_DEATH(core::CBSplineSurface Surface(-1), "");
	ASSERT_DEATH(core::CBSplineSurface Surface(0), "");
	ASSERT_NO_THROW(core::CBSplineSurface Surface(3), "");
}

TEST_F(TestBSplineSurface, DT_InvalidControlPoints)
{
	core::CBSplineSurface Surface(3);
	ASSERT_DEATH(Surface.setControlPoints(Eigen::Matrix<core::SPoint, -1, -1>()), "");

	Eigen::Matrix<core::SPoint, 3, 3> Points;
	for (int i = 0; i < 3; i++)
		for (int k = 0; k < 3; k++)
			Points.coeffRef(i, k) = core::SPoint(Eigen::Vector3f(0, 0, 0));
	ASSERT_DEATH(Surface.setControlPoints(Points), "");
}

TEST_F(TestBSplineSurface, NT_Plane)
{
	Eigen::Matrix<core::SPoint, 4, 4> ControlPoints;
	for (int i = 0; i < 4; i++)
		for (int k = 0; k < 4; k++)
			ControlPoints.coeffRef(i, k) = core::SPoint(Eigen::Vector3f(i, k, 0));
	
	core::CBSplineSurface Surface(3);
	Surface.setControlPoints(ControlPoints);

	ASSERT_LT(calcError(Surface.sample(0, 0), Eigen::Vector3f(0, 0, 0)), m_Epsilon);
	ASSERT_LT(calcError(Surface.sample(1, 1), Eigen::Vector3f(3, 3, 0)), m_Epsilon);
	ASSERT_LT(calcError(Surface.sample(0.5, 0.5), Eigen::Vector3f(1.5, 1.5, 0)), m_Epsilon);
}

TEST_F(TestBSplineSurface, NT_DrawSurfaceByOpenGL)
{
	int Rows = 5;
	int Cols = 5;
	int Depths = 5;
	int Offset = 6;
	int Degree = 3;
	int Scale = 9;

	SColor ControlPointColor(246.0f / 255.0f, 162.0f / 255.0f, 152.0f / 255.0f);
	SColor NodeColor(168.0f / 255.0f, 160.0f / 255.0f, 207.0f / 255.0f);

	Eigen::Matrix<core::SPoint, -1, -1> ControlPoints;
	generatePoints(ControlPoints, Rows, Cols, Depths);

	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	GLFWwindow* pWindow = glfwCreateWindow(G_SCRWIDTH, G_SCRHEIGHT, "BSplineSurface", NULL, NULL);
	_ASSERTE(initGL(pWindow) != -1);
	glEnable(GL_DEPTH_TEST);

	Shader Shader1("shader.vs", "shader.fs");

	std::vector<core::SPoint> Nodes;
	core::CBSplineSurface Surface(Degree);
	Surface.setControlPoints(ControlPoints);
	for (int i = 0; i <= ControlPoints.rows() * Scale; i++)
		for (int k = 0; k <= ControlPoints.cols() * Scale; k++)
			Nodes.emplace_back(Surface.sample((float)i / (float)(ControlPoints.rows() * Scale), (float)k / (float)(ControlPoints.cols() * Scale)));

	float* ControlPoints4Draw = new float[ControlPoints.size() * 6];
	float* Nodes4Draw = new float[Nodes.size() * 6];

	int ControlPointsCount = 0;
	for (int i = 0; i < ControlPoints.rows(); i++)
		for (int k = 0; k < ControlPoints.cols(); k++)
		{
			for (int m = 0; m < 3; m++)
				ControlPoints4Draw[ControlPointsCount++] = ControlPoints(i, k)[m];
			for (int m = 0; m < 3; m++)
				ControlPoints4Draw[ControlPointsCount++] = ControlPointColor[m];
		}
	std::cout << "ControlPoints Size:" << ControlPointsCount << std::endl;

	int NodesCount = -1;
	for (const auto& e : Nodes)
	{
		for (int m = 0; m < 3; m++)
			Nodes4Draw[++NodesCount] = e[m];
		for (int m = 0; m < 3; m++)
			Nodes4Draw[++NodesCount] = NodeColor[m];
	}
	std::cout << "MeshNodes Size:" << NodesCount << std::endl;

	int NodeSizePerRow = ControlPoints.rows() * Scale + 1;
	int NodeSizePerCol = ControlPoints.cols() * Scale + 1;
	int IndiceSize = (NodeSizePerRow - 1) * (NodeSizePerCol - 1) * 2 * 3;
	unsigned int* Indices = new unsigned int[IndiceSize];
	int IndicesCount = -1;
	for (int i = 0; i < NodeSizePerRow - 1; i++)
	{
		for (int k = 0; k < NodeSizePerCol; k++)
		{
			if (k != NodeSizePerCol - 1)
			{
				Indices[++IndicesCount] = i * NodeSizePerCol + k;
				Indices[++IndicesCount] = i * NodeSizePerCol + k + 1;
				Indices[++IndicesCount] = (i + 1) * NodeSizePerCol + k;
			}

			if (k != 0)
			{
				Indices[++IndicesCount] = i * NodeSizePerCol + k;
				Indices[++IndicesCount] = (i + 1) * NodeSizePerCol + k;
				Indices[++IndicesCount] = (i + 1) * NodeSizePerCol + k - 1;
			}
		}
	}
	std::cout << "NodeSizePerRow Size:" << NodeSizePerRow << std::endl;
	std::cout << "NodeSizePerCol Size:" << NodeSizePerCol << std::endl;
	std::cout << "Indices Size:" << IndicesCount << std::endl;

	unsigned int VBO, VAO;
	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);
	glBindVertexArray(VAO);

	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float) * ControlPoints.size() * 6, ControlPoints4Draw, GL_STATIC_DRAW);

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
	glEnableVertexAttribArray(1);

	unsigned int VBO2, VAO2, EBO2;
	glGenVertexArrays(1, &VAO2);
	glGenBuffers(1, &VBO2);
	glGenBuffers(1, &EBO2);
	glBindVertexArray(VAO2);

	glBindBuffer(GL_ARRAY_BUFFER, VBO2);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float) * Nodes.size() * 6, Nodes4Draw, GL_STATIC_DRAW);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO2);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * IndiceSize, Indices, GL_STATIC_DRAW);

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

		glm::mat4 Model = glm::mat4(1.0f);
		glm::mat4 View = glm::mat4(1.0f);
		glm::mat4 Projection = glm::mat4(1.0f);
		Projection = glm::perspective(glm::radians(45.0f), (float)G_SCRWIDTH / (float)G_SCRHEIGHT, 0.1f, 100.0f);
		View = glm::translate(View, glm::vec3(0.0f, 0.0f, -3.0f));
		Shader1.setMat4("projection", Projection);
		Shader1.setMat4("view", View);
		Shader1.setMat4("model", Model);

		glBindVertexArray(VAO);
		glPointSize(10.0f);
		glDrawArrays(GL_POINTS, 0, ControlPoints.size());

		glBindVertexArray(VAO2);
		glLineWidth(1.5f);
		glDrawElements(GL_TRIANGLES, IndicesCount, GL_UNSIGNED_INT, 0);

		glfwSwapBuffers(pWindow);
		glfwPollEvents();
	}

	glDeleteVertexArrays(1, &VAO);
	glDeleteVertexArrays(1, &VAO2);
	glDeleteBuffers(1, &VBO);
	glDeleteBuffers(1, &VBO2);
	glDeleteBuffers(1, &EBO2);

	glfwTerminate();
}