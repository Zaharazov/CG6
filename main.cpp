#include <SFML/Graphics.hpp>
#include <cmath>
#include <vector>
#include <iostream>
#include <limits>
#include <thread>
#include <mutex>  // Для std::mutex и std::lock_guard
#include <omp.h>
#include <fstream>
#include <sstream>
#include "tiny_obj_loader.h" 

#define M_PI 3.14159265358979323846

const int WIDTH = 600;
const int HEIGHT = 500;

// вектор для 3D операций
struct Vector3
{
	float x, y, z; // координаты

	Vector3() : x(0), y(0), z(0) {} // конструкторы
	Vector3(float x, float y, float z) : x(x), y(y), z(z) {}
	// перегрузка операторов
	Vector3 operator+(const Vector3& v) const { return Vector3(x + v.x, y + v.y, z + v.z); }
	Vector3 operator-(const Vector3& v) const { return Vector3(x - v.x, y - v.y, z - v.z); }
	Vector3 operator*(float s) const { return Vector3(x * s, y * s, z * s); }
	Vector3 operator*(const Vector3& v) const { return Vector3(x * v.x, y * v.y, z * v.z); } // Покомпонентное умножение
	Vector3 operator/(float s) const { return Vector3(x / s, y / s, z / s); }
	// скалярное произведенеие
	float dot(const Vector3& v) const { return x * v.x + y * v.y + z * v.z; }
	// нормализует вектор
	Vector3 normalize() const
	{
		float len = std::sqrt(x * x + y * y + z * z);
		return *this / len;
	}
	// векторное произведение
	Vector3 cross(const Vector3& v) const
	{
		return Vector3(
			y * v.z - z * v.y,
			z * v.x - x * v.z,
			x * v.y - y * v.x
		);
	}
};

struct Triangle {
	Vector3 v0, v1, v2;      // Вершины треугольника
	Vector3 normal;          // Нормаль к треугольнику
	Vector3 color;           // Цвет треугольника
	float reflectivity;      // Коэффициент отражения
	float transmissivity;    // Коэффициент прозрачности
	float refractiveIndex;   // Коэффициент преломления

	// Конструктор с параметрами
	Triangle(const Vector3& v0, const Vector3& v1, const Vector3& v2,
		const Vector3& col, float refl, float trans = 0.0f, float refrIdx = 1.0f)
		: v0(v0), v1(v1), v2(v2), color(col), reflectivity(refl),
		transmissivity(trans), refractiveIndex(refrIdx)
	{
		// Вычисляем нормаль
		Vector3 edge1 = v1 - v0;
		Vector3 edge2 = v2 - v0;
		normal = edge1.cross(edge2).normalize();
	}

	// Метод пересечения луча с треугольником
	bool intersect(const Vector3& origin, const Vector3& dir, float& t) const {
		const float epsilon = 1e-6; // Погрешность для вычислений

		Vector3 edge1 = v1 - v0;
		Vector3 edge2 = v2 - v0;
		Vector3 h = dir.cross(edge2);
		float a = edge1.dot(h);

		// Проверяем, параллелен ли луч треугольнику
		if (std::abs(a) < epsilon) return false;

		float f = 1.0f / a;
		Vector3 s = origin - v0;
		float u = f * s.dot(h);

		// Проверяем, находится ли точка пересечения внутри треугольника
		if (u < 0.0 || u > 1.0) return false;

		Vector3 q = s.cross(edge1);
		float v = f * dir.dot(q);
		if (v < 0.0 || u + v > 1.0) return false;

		// Вычисляем параметр t для точки пересечения
		t = f * edge2.dot(q);
		return t > epsilon; // Убедимся, что пересечение происходит впереди луча
	}
};

bool loadOBJ(const std::string& filepath, std::vector<Triangle>& triangles, const Vector3& color) {
	std::ifstream file(filepath);
	if (!file.is_open()) {
		std::cerr << "Error: Cannot open file " << filepath << std::endl;
		return false;
	}

	std::vector<Vector3> vertices;
	std::string line;
	while (std::getline(file, line)) {
		std::istringstream iss(line);
		std::string prefix;
		iss >> prefix;
		if (prefix == "v") {
			float x, y, z;
			iss >> x >> y >> z;
			vertices.emplace_back(x, y, z);
		}
		else if (prefix == "f") {
			int v0, v1, v2;
			char slash; // для игнорирования символов '/'
			iss >> v0 >> slash >> v1 >> slash >> v2;
			triangles.emplace_back(vertices[v0 - 1], vertices[v1 - 1], vertices[v2 - 1], color, 0.2f);

		}
	}

	file.close();
	return true;
}

// сфера
struct Sphere
{
	Vector3 center;
	float radius;
	Vector3 color;
	float reflectivity; // отражение
	float transmissivity; // прозрачность
	float refractiveIndex; // показатель преломления
	// конструктор
	Sphere(const Vector3& c, float r, const Vector3& col, float refl, float trans, float refrIdx)
		: center(c), radius(r), color(col), reflectivity(refl), transmissivity(trans), refractiveIndex(refrIdx) {}
	// проверяем, пересекает ли луч сферу
	bool intersect(const Vector3& origin, const Vector3& direction, float& t) const
	{
		Vector3 oc = origin - center;
		float b = oc.dot(direction);
		float c = oc.dot(oc) - radius * radius;
		float discriminant = b * b - c;

		// Ранний выход
		if (discriminant < 0) return false;

		float sqrtD = std::sqrt(discriminant);
		float t0 = -b - sqrtD;
		float t1 = -b + sqrtD;

		// Выбираем ближайшую точку
		if (t0 > 1e-4)
			t = t0;
		else if (t1 > 1e-4)
			t = t1;
		else
			return false;

		return true;
	}
};

// плоскость
struct Plane
{
	Vector3 point; // исходная точка
	Vector3 normal; // ориентация
	Vector3 color;
	float reflectivity; // отражжение
	// конструктор
	Plane(const Vector3& p, const Vector3& n, const Vector3& col, float refl)
		: point(p), normal(n.normalize()), color(col), reflectivity(refl) {}
	// аналогичная функция
	bool intersect(const Vector3& origin, const Vector3& direction, float& t) const
	{
		// Находим скалярное произведение с нормалью
		float denom = normal.dot(direction);

		// Проверяем, не параллелен ли луч плоскости
		if (denom < 1e-6 && denom > -1e-6)
			return false;

		// Вычисление пересечения
		Vector3 p0l0 = point - origin;
		t = p0l0.dot(normal) / denom;

		// Ранний выход: пересечение перед началом луча
		return t >= 0;
	}

};

// куб
struct Cube
{
	Vector3 min; // две противоположные вершины куба
	Vector3 max;
	Vector3 color;
	float reflectivity; // отражение
	float transmissivity; // прозрачность
	float refractiveIndex; // показатель преломления
	// конструктор
	Cube(const Vector3& min, const Vector3& max, const Vector3& col, float refl, float trans, float refrIdx)
		: min(min), max(max), color(col), reflectivity(refl), transmissivity(trans), refractiveIndex(refrIdx) {}
	// аналогичная функция пересечения луча с кубом
	bool intersect(const Vector3& origin, const Vector3& direction, float& t) const
	{
		// Параметры для быстрого выхода
		float tmin, tmax, tymin, tymax, tzmin, tzmax;

		// Проверка по осям X
		tmin = (min.x - origin.x) / direction.x;
		tmax = (max.x - origin.x) / direction.x;
		if (tmin > tmax) std::swap(tmin, tmax);

		// Проверка по осям Y
		tymin = (min.y - origin.y) / direction.y;
		tymax = (max.y - origin.y) / direction.y;
		if (tymin > tymax) std::swap(tymin, tymax);

		// Проверка на выход за пределы
		if (tmin > tymax || tymin > tmax) return false;
		if (tymin > tmin) tmin = tymin;
		if (tymax < tmax) tmax = tymax;

		// Проверка по осям Z
		tzmin = (min.z - origin.z) / direction.z;
		tzmax = (max.z - origin.z) / direction.z;
		if (tzmin > tzmax) std::swap(tzmin, tzmax);

		// Проверка на выход за пределы
		if (tmin > tzmax || tzmin > tmax) return false;
		if (tzmin > tmin) tmin = tzmin;
		if (tzmax < tmax) tmax = tzmax;

		t = tmin;
		return true;
	}

};

// Источник света
struct Light
{
	Vector3 position;
	Vector3 intensity;

	Light(const Vector3& pos, const Vector3& inten) : position(pos), intensity(inten) {}
};

// камера
struct Camera
{
	Vector3 position;
	// ориентация
	Vector3 forward;
	Vector3 up;
	Vector3 right;

	// конструктор
	Camera(const Vector3& pos, const Vector3& lookAt, const Vector3& upVec)
		: position(pos)
	{
		forward = (lookAt - position).normalize();
		right = forward.cross(upVec).normalize();
		up = right.cross(forward);
	}
	// считаем направление луча из камеры для пикселя на изображении
	Vector3 getRayDirection(float x, float y, float imageWidth, float imageHeight) const
	{
		float fovScale = tan(M_PI / 4); // масштаб поля зрения
		float aspectRatio = imageWidth / imageHeight; // соотношение сторон
		// преобразуем пиксельные координаты в пространственные
		float px = (2 * (x + 0.5) / imageWidth - 1) * aspectRatio * fovScale;
		float py = (1 - 2 * (y + 0.5) / imageHeight) * fovScale;
		return (forward + right * px + up * py).normalize(); // вектор направления
	}

	void lookAtCursor(int cursorX, int cursorY, int imageWidth, int imageHeight)
	{
		// Получаем направление луча от камеры в сторону координат курсора
		Vector3 direction = getRayDirection(cursorX, cursorY, imageWidth, imageHeight);

		// Поворот камеры на небольшое значение, контролируемое коэффициентом
		float smoothingFactor = 0.1f; // Коэффициент замедления, чем меньше - тем медленнее камера

		// Поворачиваем камеру так, чтобы она смотрела в эту сторону с учетом smoothingFactor
		forward = forward * (1 - smoothingFactor) + direction * smoothingFactor;

		// Нормализуем направление
		forward = forward.normalize();

		// Сохраняем вертикальное направление (up) неизменным
		// Чтобы избежать наклона по вертикали, пересчитываем right
		right = forward.cross(up).normalize();

		// Переопределяем up, чтобы она была перпендикулярна направлению forward и right
		//up = right.cross(forward);

		// Таким образом, мы гарантируем, что вертикальное направление (up) не будет изменяться
	}

};

// преломление
Vector3 refract(const Vector3& I, const Vector3& N, float eta)
{ // вектор падения, вектор нормали к поверхности, отношение преломлений сред -> вектор направления преломленного луча
	float cosI = -I.dot(N); // косинус угла падения
	float sinT2 = eta * eta * (1 - cosI * cosI);
	if (sinT2 > 1) return Vector3(0, 0, 0); // проверка на полное внутренне отражение
	float cosT = std::sqrt(1 - sinT2); // косинус угла преломления
	return I * eta + N * (eta * cosI - cosT); // корректный вектор преломленного луча
}

// трассировка луча
Vector3 traceRay(const Vector3& origin, const Vector3& direction,
	const std::vector<Sphere>& spheres, const std::vector<Plane>& planes,
	const std::vector<Cube>& cubes, const std::vector<Light>& lights,
	const std::vector<Triangle>& triangles, int depth)
{
	if (depth <= 0) return Vector3(0, 0, 0); // черный цвет при нулевой глубине

	float tMin = std::numeric_limits<float>::infinity(); // минимальная дистанция до пересечения
	Vector3 hitPoint, normal, baseColor; // точка пересечения луча с объектом, нормаль и цвет в ней
	float reflectivity = 0; // отражение
	float transmissivity = 0; // прозрачность
	float refractiveIndex = 1; // преломление

	// Проверка пересечений с объектами сцены (сферы, плоскости, кубы)

	// Сферы
	for (const auto& sphere : spheres)
	{
		float t;
		if (sphere.intersect(origin, direction, t) && t < tMin)
		{
			tMin = t;
			hitPoint = origin + direction * t;
			normal = (hitPoint - sphere.center).normalize();
			baseColor = sphere.color;
			reflectivity = sphere.reflectivity;
			transmissivity = sphere.transmissivity;
			refractiveIndex = sphere.refractiveIndex;
		}
	}

	// Плоскости
	for (const auto& plane : planes)
	{
		float t;
		if (plane.intersect(origin, direction, t) && t < tMin)
		{
			tMin = t;
			hitPoint = origin + direction * t;
			normal = plane.normal;
			baseColor = plane.color;
			reflectivity = plane.reflectivity;
		}
	}

	// Кубы
	for (const auto& cube : cubes)
	{
		float t;
		if (cube.intersect(origin, direction, t) && t < tMin)
		{
			tMin = t;
			hitPoint = origin + direction * t;

			// Определение нормали
			if (std::abs(hitPoint.x - cube.min.x) < 1e-3) normal = Vector3(-1, 0, 0);
			else if (std::abs(hitPoint.x - cube.max.x) < 1e-3) normal = Vector3(1, 0, 0);
			else if (std::abs(hitPoint.y - cube.min.y) < 1e-3) normal = Vector3(0, -1, 0);
			else if (std::abs(hitPoint.y - cube.max.y) < 1e-3) normal = Vector3(0, 1, 0);
			else if (std::abs(hitPoint.z - cube.min.z) < 1e-3) normal = Vector3(0, 0, -1);
			else if (std::abs(hitPoint.z - cube.max.z) < 1e-3) normal = Vector3(0, 0, 1);

			baseColor = cube.color;
			reflectivity = cube.reflectivity;
			transmissivity = cube.transmissivity;
			refractiveIndex = cube.refractiveIndex;
		}
	}

	// Треугольники из .obj
	for (const auto& triangle : triangles)
	{
		float t;
		if (triangle.intersect(origin, direction, t) && t < tMin)
		{
			tMin = t;
			hitPoint = origin + direction * t;
			normal = triangle.normal;
			baseColor = triangle.color;
			reflectivity = triangle.reflectivity;
			transmissivity = triangle.transmissivity;
			refractiveIndex = triangle.refractiveIndex;
		}
	}

	if (tMin == std::numeric_limits<float>::infinity()) return Vector3(0, 0, 0); // ничего не пересечено

	// Освещение
	Vector3 color(0, 0, 0);
	for (const auto& light : lights)
	{
		Vector3 lightDir = (light.position - hitPoint).normalize();
		Vector3 lightColor = baseColor * std::max(0.f, normal.dot(lightDir));
		color = color + lightColor * light.intensity;
	}

	// Отражение
	if (reflectivity > 0)
	{
		Vector3 reflectDir = direction - normal * 2 * direction.dot(normal);
		color = color + traceRay(hitPoint + normal * 1e-4, reflectDir, spheres, planes, cubes, lights, triangles, depth - 1) * reflectivity;
	}

	// Преломление
	if (transmissivity > 0)
	{
		float eta = direction.dot(normal) < 0 ? 1 / refractiveIndex : refractiveIndex;
		Vector3 refractDir = refract(direction, normal, eta);
		color = color + traceRay(hitPoint - normal * 1e-4, refractDir, spheres, planes, cubes, lights, triangles, depth - 1) * transmissivity;
	}

	return color;
}

std::mutex sceneMutex;

// Функция для добавления нового куба в сцену
void addCube(std::vector<Cube>& cubes, const Camera& camera)
{
	std::lock_guard<std::mutex> lock(sceneMutex); // Защищаем критическую секцию
	// Создаем куб на расстоянии 3 единицы от камеры вдоль ее взгляда
	Vector3 position = camera.position + camera.forward * 4.0f; // Расстояние от камеры
	Cube newCube(position - Vector3(0.5f, 0.5f, 0.5f), position + Vector3(0.5f, 0.5f, 0.5f), Vector3(1, 0, 0), 0.2, 0.3, 1.0);
	cubes.push_back(newCube);
}

void addSphere(std::vector<Sphere>& spheres, const Camera& camera)
{
	std::lock_guard<std::mutex> lock(sceneMutex); // Защищаем критическую секцию
	// Создаем сферу на расстоянии 3 единицы от камеры вдоль ее взгляда
	Vector3 position = camera.position + camera.forward * 4.0f; // Расстояние от камеры
	Sphere newSphere(position, 0.7f, Vector3(0, 0, 1), 0.2f, 0.3f, 1.0f);
	spheres.push_back(newSphere);
}

void addLight(std::vector<Light>& lights, const Camera& camera)
{
	std::lock_guard<std::mutex> lock(sceneMutex); // Защищаем критическую секцию
	// Создаем новый источник света в точке, куда направлена камера
	Vector3 position = camera.position + camera.forward * 3.0f; // Расстояние от камеры
	Light newLight(position, Vector3(0.8, 0.8, 0.8)); // Белый свет
	lights.push_back(newLight);
}

// Функция отрисовки строк изображения
void renderRow(int startY, int endY, sf::Image& image, const Camera& camera,
	const std::vector<Sphere>& spheres, const std::vector<Plane>& planes,
	const std::vector<Cube>& cubes, const std::vector<Light>& lights,
	const std::vector<Triangle>& triangles, int traceDepth)
{
#pragma omp parallel for
	for (int y = startY; y < endY; ++y)
	{
		for (int x = 0; x < WIDTH; ++x)
		{
			// Получаем направление луча из камеры
			Vector3 direction = camera.getRayDirection(x, y, WIDTH, HEIGHT);
			// Трассируем луч
			Vector3 color = traceRay(camera.position, direction, spheres, planes, cubes, lights, triangles, traceDepth);

			// Преобразуем цвет в формат для пикселя
			sf::Color pixelColor(
				std::min(255, static_cast<int>(color.x * 255)),
				std::min(255, static_cast<int>(color.y * 255)),
				std::min(255, static_cast<int>(color.z * 255))
			);

			image.setPixel(x, y, pixelColor); // Устанавливаем пиксель
		}
	}
}


std::vector<std::thread> threadPool;

void renderImageParallelOptimized(
	sf::Image& image, const Camera& camera,
	const std::vector<Sphere>& spheres, const std::vector<Plane>& planes,
	const std::vector<Cube>& cubes, const std::vector<Light>& lights,
	const std::vector<Triangle>& triangles, // Добавлено
	int traceDepth, int WIDTH, int HEIGHT)
{
	// Создаем пул потоков
	int numThreads = std::thread::hardware_concurrency();
	int rowsPerThread = HEIGHT / numThreads;

	std::vector<std::thread> threadPool;

	// Распределяем рендеринг по потокам
	for (int i = 0; i < numThreads; ++i)
	{
		int startY = i * rowsPerThread;
		int endY = (i == numThreads - 1) ? HEIGHT : (i + 1) * rowsPerThread;

		threadPool.push_back(std::thread(renderRow, startY, endY, std::ref(image),
			std::cref(camera), std::cref(spheres), std::cref(planes),
			std::cref(cubes), std::cref(lights), std::cref(triangles), traceDepth));
	}

	// Ожидаем завершения всех потоков
	for (auto& thread : threadPool)
	{
		if (thread.joinable())
			thread.join();
	}
}


// основной рендеринг
int main()
{
	int traceDepth = 1;

	sf::RenderWindow window(sf::VideoMode(WIDTH, HEIGHT), "Optimized Scene Rendering");
	sf::Image image;
	image.create(WIDTH, HEIGHT);

	std::vector<Sphere> spheres = {};
	std::vector<Plane> planes = {
		Plane(Vector3(0, -2, 0), Vector3(0, 1, 0), Vector3(0.8, 0.8, 0.8), 0.3),
	};
	std::vector<Cube> cubes = {};
	std::vector<Light> lights = {};
	std::vector<Triangle> triangles; // Для треугольников из OBJ

	// Загрузка OBJ-файла
	//std::string inputfile = "cube.obj";
	//std::string inputfile = "dodecahedron.obj";
	tinyobj::attrib_t attrib;
	std::vector<tinyobj::shape_t> shapes;
	//std::vector<tinyobj::material_t> materials;
	//std::string warn, err;

	//if (!tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, inputfile.c_str())) {
	//	std::cerr << "Ошибка загрузки OBJ: " << warn << err << std::endl;
	//	return 1;
	//}

	// Преобразование данных из OBJ в треугольники
	for (const auto& shape : shapes) {
		for (size_t i = 0; i < shape.mesh.indices.size(); i += 3) {
			tinyobj::index_t idx0 = shape.mesh.indices[i];
			tinyobj::index_t idx1 = shape.mesh.indices[i + 1];
			tinyobj::index_t idx2 = shape.mesh.indices[i + 2];

			Vector3 v0(attrib.vertices[3 * idx0.vertex_index],
				attrib.vertices[3 * idx0.vertex_index + 1],
				attrib.vertices[3 * idx0.vertex_index + 2]);
			Vector3 v1(attrib.vertices[3 * idx1.vertex_index],
				attrib.vertices[3 * idx1.vertex_index + 1],
				attrib.vertices[3 * idx1.vertex_index + 2]);
			Vector3 v2(attrib.vertices[3 * idx2.vertex_index],
				attrib.vertices[3 * idx2.vertex_index + 1],
				attrib.vertices[3 * idx2.vertex_index + 2]);

			// Задаем треугольник с произвольным цветом и отражением
			Vector3 color(0.6, 0.6, 0.6);
			float reflection = 0.2f;
			triangles.emplace_back(v0, v1, v2, color, reflection);
		}
	}

	Camera camera(Vector3(0, 2, -0.5), Vector3(-1, 2, -0.5), Vector3(0, 1, 0));

	renderImageParallelOptimized(image, camera, spheres, planes, cubes, lights, triangles, traceDepth, WIDTH, HEIGHT);

	sf::Texture texture;
	texture.loadFromImage(image);
	sf::Sprite sprite(texture);

	// Полоска FPS
	sf::RectangleShape fpsBar(sf::Vector2f(200.f, 20.f));
	fpsBar.setPosition(10.f, 10.f);

	sf::Clock clock;
	int turn = 0, RTturn = 0;

	while (window.isOpen()) {
		sf::Event event;
		while (window.pollEvent(event)) {
			if (event.type == sf::Event::Closed) {
				window.close();
			}
			if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::C) {
				if (turn % 2 == 0)
					addCube(cubes, camera); // Добавляем куб
				else
					addSphere(spheres, camera); // Добавляем сферу
				turn++;

				renderImageParallelOptimized(image, camera, spheres, planes, cubes, lights, triangles, traceDepth, WIDTH, HEIGHT);

				texture.loadFromImage(image);
			}

			if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::R) {
				RTturn++;

				traceDepth = (RTturn % 2 == 0) ? 1 : 4;

				renderImageParallelOptimized(image, camera, spheres, planes, cubes, lights, triangles, traceDepth, WIDTH, HEIGHT);

				texture.loadFromImage(image);
			}

			if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::L) {
				addLight(lights, camera); // Добавляем источник света

				renderImageParallelOptimized(image, camera, spheres, planes, cubes, lights, triangles, traceDepth, WIDTH, HEIGHT);

				texture.loadFromImage(image);
			}

			if (event.type == sf::Event::MouseMoved) {
				camera.lookAtCursor(event.mouseMove.x, event.mouseMove.y, WIDTH, HEIGHT);

				renderImageParallelOptimized(image, camera, spheres, planes, cubes, lights, triangles, traceDepth, WIDTH, HEIGHT);

				texture.loadFromImage(image);
				sprite.setTexture(texture);

				window.clear();
				window.draw(sprite);
				window.draw(fpsBar);
				window.display();
			}
		}

		float cameraSpeed = 0.5f;
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::W)) {
			camera.position = camera.position + camera.forward * cameraSpeed;
		}
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::S)) {
			camera.position = camera.position - camera.forward * cameraSpeed;
		}
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::A)) {
			camera.position = camera.position - camera.right * cameraSpeed;
		}
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::D)) {
			camera.position = camera.position + camera.right * cameraSpeed;
		}

		float elapsedTime = clock.restart().asSeconds();
		float fps = 1.f / elapsedTime;

		float normalizedFps = std::min(fps / 60.f, 1.f);
		fpsBar.setSize(sf::Vector2f(200.f * normalizedFps, 20.f));

		if (fps >= 50)
			fpsBar.setFillColor(sf::Color::Green);
		else if (fps >= 20)
			fpsBar.setFillColor(sf::Color::Yellow);
		else
			fpsBar.setFillColor(sf::Color::Red);

		renderImageParallelOptimized(image, camera, spheres, planes, cubes, lights, triangles, traceDepth, WIDTH, HEIGHT);
		texture.loadFromImage(image);
		sprite.setTexture(texture);

		window.clear();
		window.draw(sprite);
		window.draw(fpsBar);
		window.display();
	}

	return 0;
}
