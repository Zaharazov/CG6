#include <SFML/Graphics.hpp>  // Для графической библиотеки SFML
#include <cmath>  // Для математических операций, таких как sqrt
#include <vector>  // Для использования динамических массивов
#include <iostream>  // Для вывода сообщений в консоль
#include <limits>  // Для работы с пределами значений типов данных
#include <thread>  // Для многозадачности с использованием потоков
#include <mutex>  // Для синхронизации потоков (std::mutex и std::lock_guard)
#include <omp.h>  // Для параллельных вычислений с использованием OpenMP
#include <fstream>  // Для работы с файлами
#include <sstream>  // Для работы со строками
#include "tiny_obj_loader.h"  // Библиотека для загрузки 3D объектов в формате .obj

#define M_PI 3.14159265358979323846  // Определение числа Пи

const int WIDTH = 600;  // Устанавливаем ширину экрана (в пикселях)
const int HEIGHT = 500; // Устанавливаем высоту экрана (в пикселях)

// Структура для представления 3D вектора (например, координат в пространстве)
struct Vector3
{
	float x, y, z; // Компоненты вектора (координаты)

	// Конструктор по умолчанию, инициализирует все компоненты нулями
	Vector3() : x(0), y(0), z(0) {}

	// Конструктор с параметрами для задания координат вектора
	Vector3(float x, float y, float z) : x(x), y(y), z(z) {}

	// Перегрузка оператора сложения для двух векторов
	Vector3 operator+(const Vector3& v) const {
		return Vector3(x + v.x, y + v.y, z + v.z);
	}

	// Перегрузка оператора вычитания для двух векторов
	Vector3 operator-(const Vector3& v) const {
		return Vector3(x - v.x, y - v.y, z - v.z);
	}

	// Перегрузка оператора умножения на скаляр
	Vector3 operator*(float s) const {
		return Vector3(x * s, y * s, z * s);
	}

	// Перегрузка оператора покомпонентного умножения двух векторов
	Vector3 operator*(const Vector3& v) const {
		return Vector3(x * v.x, y * v.y, z * v.z);
	}

	// Перегрузка оператора деления на скаляр
	Vector3 operator/(float s) const {
		return Vector3(x / s, y / s, z / s);
	}

	// Скалярное произведение (dot product) двух векторов
	float dot(const Vector3& v) const {
		return x * v.x + y * v.y + z * v.z;
	}

	// Нормализация вектора, то есть приведение его длины к единице
	Vector3 normalize() const
	{
		float len = std::sqrt(x * x + y * y + z * z); // Находим длину вектора
		return *this / len;  // Возвращаем нормализованный вектор (единичной длины)
	}

	// Векторное произведение (cross product) двух векторов
	Vector3 cross(const Vector3& v) const
	{
		return Vector3(
			y * v.z - z * v.y,
			z * v.x - x * v.z,
			x * v.y - y * v.x
		);
	}
};

// Структура для треугольника
struct Triangle {
	Vector3 v0, v1, v2;      // Вершины треугольника
	Vector3 normal;          // Нормаль к треугольнику (перпендикуляр к его поверхности)
	Vector3 color;           // Цвет треугольника
	float reflectivity;      // Коэффициент отражения (определяет, насколько объект отражает свет)
	float transmissivity;    // Коэффициент прозрачности (определяет, насколько объект прозрачен)
	float refractiveIndex;   // Коэффициент преломления (для моделирования преломления света)

	// Конструктор, который инициализирует все параметры треугольника
	Triangle(const Vector3& v0, const Vector3& v1, const Vector3& v2,
		const Vector3& col, float refl, float trans = 0.0f, float refrIdx = 1.0f)
		: v0(v0), v1(v1), v2(v2), color(col), reflectivity(refl),
		transmissivity(trans), refractiveIndex(refrIdx)
	{
		// Вычисление нормали треугольника с помощью векторного произведения его рёбер
		Vector3 edge1 = v1 - v0;  // Вектор от v0 до v1
		Vector3 edge2 = v2 - v0;  // Вектор от v0 до v2
		normal = edge1.cross(edge2).normalize(); // Нормаль = векторное произведение рёбер
	}

	// Метод для вычисления пересечения луча с треугольником
	bool intersect(const Vector3& origin, const Vector3& dir, float& t) const {
		const float epsilon = 1e-6; // Погрешность для вычислений (чтобы избежать деления на ноль)

		Vector3 edge1 = v1 - v0;  // Ребро треугольника от v0 до v1
		Vector3 edge2 = v2 - v0;  // Ребро треугольника от v0 до v2
		Vector3 h = dir.cross(edge2);  // Векторное произведение направления луча и второго ребра
		float a = edge1.dot(h);  // Скалярное произведение первого ребра с h

		// Если a близко к нулю, луч параллелен плоскости треугольника
		if (std::abs(a) < epsilon) return false;

		float f = 1.0f / a;  // Обратная величина a
		Vector3 s = origin - v0;  // Вектор от вершины треугольника до начала луча
		float u = f * s.dot(h);  // Коэффициент u для проверки попадания в треугольник

		// Проверка, находится ли точка пересечения внутри треугольника
		if (u < 0.0 || u > 1.0) return false;

		Vector3 q = s.cross(edge1);  // Векторное произведение s и первого ребра
		float v = f * dir.dot(q);  // Коэффициент v для проверки попадания в треугольник
		if (v < 0.0 || u + v > 1.0) return false;

		// Вычисление параметра t для точности пересечения
		t = f * edge2.dot(q);
		return t > epsilon;  // Проверка, что пересечение происходит впереди луча
	}
};

// Функция для загрузки 3D объекта в формате .obj
bool loadOBJ(const std::string& filepath, std::vector<Triangle>& triangles, const Vector3& color) {
	std::ifstream file(filepath);  // Открываем файл для чтения
	if (!file.is_open()) {  // Если файл не открылся, выводим ошибку
		std::cerr << "Error: Cannot open file " << filepath << std::endl;
		return false;
	}

	std::vector<Vector3> vertices;  // Список для хранения вершин объекта
	std::string line;  // Строка для чтения каждой строки из файла
	while (std::getline(file, line)) {  // Чтение строк из файла
		std::istringstream iss(line);  // Создаем поток для обработки строки
		std::string prefix;  // Префикс строки (например, "v" для вершин)
		iss >> prefix;
		if (prefix == "v") {  // Если строка содержит вершины
			float x, y, z;
			iss >> x >> y >> z;  // Чтение координат вершины
			vertices.emplace_back(x, y, z);  // Добавление вершины в список
		}
		else if (prefix == "f") {  // Если строка содержит грани (треугольники)
			int v0, v1, v2;
			char slash;  // Для игнорирования символов '/'
			iss >> v0 >> slash >> v1 >> slash >> v2;  // Чтение индексов вершин
			triangles.emplace_back(vertices[v0 - 1], vertices[v1 - 1], vertices[v2 - 1], color, 0.2f);
			// Добавление треугольника в список, используя индексы вершин
		}
	}

	file.close();  // Закрытие файла
	return true;  // Успешная загрузка
}

// Структура для представления сферы
struct Sphere
{
	Vector3 center;  // Центр сферы
	float radius;    // Радиус сферы
	Vector3 color;   // Цвет сферы
	float reflectivity; // Коэффициент отражения
	float transmissivity; // Коэффициент прозрачности
	float refractiveIndex; // Показатель преломления

	// Конструктор для инициализации сферы
	Sphere(const Vector3& c, float r, const Vector3& col, float refl, float trans, float refrIdx)
		: center(c), radius(r), color(col), reflectivity(refl), transmissivity(trans), refractiveIndex(refrIdx) {}

	// Метод для проверки пересечения луча с поверхностью сферы
	bool intersect(const Vector3& origin, const Vector3& direction, float& t) const
	{
		Vector3 oc = origin - center;  // Вектор от центра сферы до начала луча
		float b = oc.dot(direction);   // Скалярное произведение вектора от центра до начала луча с направлением луча
		float c = oc.dot(oc) - radius * radius;  // Расстояние от луча до центра сферы
		float discriminant = b * b - c;  // Дискриминант для вычисления пересечений

		// Если дискриминант меньше 0, пересечений нет
		if (discriminant < 0) return false;

		float sqrtD = std::sqrt(discriminant);  // Корень из дискриминанта
		float t0 = -b - sqrtD;  // Первая возможная точка пересечения
		float t1 = -b + sqrtD;  // Вторая возможная точка пересечения

		// Выбираем ближайшую точку пересечения
		if (t0 > 1e-4)
			t = t0;
		else if (t1 > 1e-4)
			t = t1;
		else
			return false;

		return true;
	}
};

// Структура для представления плоскости
struct Plane
{
	Vector3 point;  // Точка, через которую проходит плоскость
	Vector3 normal; // Нормаль плоскости
	Vector3 color;  // Цвет плоскости
	float reflectivity; // Коэффициент отражения

	// Конструктор для инициализации плоскости
	Plane(const Vector3& p, const Vector3& n, const Vector3& col, float refl)
		: point(p), normal(n.normalize()), color(col), reflectivity(refl) {}

	// Метод для проверки пересечения луча с плоскостью
	bool intersect(const Vector3& origin, const Vector3& direction, float& t) const
	{
		// Скалярное произведение нормали с направлением луча
		float denom = normal.dot(direction);

		// Проверка, параллелен ли луч плоскости (если скалярное произведение близко к нулю)
		if (denom < 1e-6 && denom > -1e-6)
			return false;

		// Вычисление расстояния от начала луча до пересечения
		Vector3 p0l0 = point - origin;
		t = p0l0.dot(normal) / denom;

		// Пересечение не может происходить до начала луча
		return t >= 0;
	}
};
// Структура для представления куба
struct Cube
{
	Vector3 min;  // Одна из противоположных вершин куба (минимальная точка)
	Vector3 max;  // Другая противоположная вершина куба (максимальная точка)
	Vector3 color; // Цвет куба
	float reflectivity; // Коэффициент отражения (определяет, насколько объект отражает свет)
	float transmissivity; // Коэффициент прозрачности (определяет, насколько объект прозрачен)
	float refractiveIndex; // Коэффициент преломления (определяет, как свет будет преломляться через объект)

	// Конструктор для инициализации всех параметров куба
	Cube(const Vector3& min, const Vector3& max, const Vector3& col, float refl, float trans, float refrIdx)
		: min(min), max(max), color(col), reflectivity(refl), transmissivity(trans), refractiveIndex(refrIdx) {}

	// Метод для пересечения луча с кубом (вычисление, пересекает ли луч куб)
	bool intersect(const Vector3& origin, const Vector3& direction, float& t) const
	{
		// Параметры для быстрого выхода
		float tmin, tmax, tymin, tymax, tzmin, tzmax;

		// Проверка пересечения по оси X
		tmin = (min.x - origin.x) / direction.x;
		tmax = (max.x - origin.x) / direction.x;
		if (tmin > tmax) std::swap(tmin, tmax);  // Если tmin больше tmax, меняем их местами

		// Проверка пересечения по оси Y
		tymin = (min.y - origin.y) / direction.y;
		tymax = (max.y - origin.y) / direction.y;
		if (tymin > tymax) std::swap(tymin, tymax);  // Если tymin больше tymax, меняем их местами

		// Проверка на выход за пределы (пересечение должно быть на всех осях)
		if (tmin > tymax || tymin > tmax) return false; // Если пересечение не происходит, возвращаем false
		if (tymin > tmin) tmin = tymin;  // Обновляем tmin, если необходимо
		if (tymax < tmax) tmax = tymax;  // Обновляем tmax, если необходимо

		// Проверка пересечения по оси Z
		tzmin = (min.z - origin.z) / direction.z;
		tzmax = (max.z - origin.z) / direction.z;
		if (tzmin > tzmax) std::swap(tzmin, tzmax);  // Если tzmin больше tzmax, меняем их местами

		// Проверка на выход за пределы
		if (tmin > tzmax || tzmin > tmax) return false;  // Если пересечение не происходит, возвращаем false
		if (tzmin > tmin) tmin = tzmin;  // Обновляем tmin
		if (tzmax < tmax) tmax = tzmax;  // Обновляем tmax

		t = tmin;  // Возвращаем точку пересечения
		return true;  // Возвращаем true, если пересечение было найдено
	}
};

// Структура для источника света
struct Light
{
	Vector3 position;   // Позиция источника света в 3D-пространстве
	Vector3 intensity;  // Интенсивность света (например, цвет и яркость)
	Vector3 direction;  // Направление света (для направленного света или прожекторов)

	// Конструктор для точечного источника света (с позицией)
	Light(const Vector3& pos, const Vector3& inten)
		: position(pos), intensity(inten), direction(Vector3(0, 0, 0)) {}

	// Конструктор для направленного источника света (с позицией и направлением)
	Light(const Vector3& pos, const Vector3& inten, const Vector3& dir)
		: position(pos), intensity(inten), direction(dir) {}
};

// Структура для камеры, которая используется для рендеринга сцены
struct Camera
{
	Vector3 position;   // Позиция камеры в 3D-пространстве
	Vector3 forward;    // Направление, в котором смотрит камера (вектор вперед)
	Vector3 up;         // Направление вверх для камеры (перпендикулярно направлению вперед)
	Vector3 right;      // Направление вправо для камеры (перпендикулярно forward и up)

	// Конструктор камеры, инициализирует положение и направления
	Camera(const Vector3& pos, const Vector3& lookAt, const Vector3& upVec)
		: position(pos)
	{
		forward = (lookAt - position).normalize();  // Направление от камеры к цели (normalized)
		right = forward.cross(upVec).normalize();  // Вектор вправо, пересекающий направление вперед и up
		up = right.cross(forward);  // Направление вверх, пересекающее right и forward
	}

	// Метод для получения направления луча из камеры на пиксель на изображении
	Vector3 getRayDirection(float x, float y, float imageWidth, float imageHeight) const
	{
		// Масштаб поля зрения (поля зрения камеры)
		float fovScale = tan(M_PI / 4);  // В данном случае угол поля зрения 45°
		float aspectRatio = imageWidth / imageHeight;  // Соотношение сторон изображения

		// Преобразуем пиксельные координаты в пространственные
		float px = (2 * (x + 0.5) / imageWidth - 1) * aspectRatio * fovScale;
		float py = (1 - 2 * (y + 0.5) / imageHeight) * fovScale;

		// Направление луча из камеры в точку на изображении
		return (forward + right * px + up * py).normalize();
	}

	// Метод для плавного поворота камеры в сторону курсора
	void lookAtCursor(int cursorX, int cursorY, int imageWidth, int imageHeight)
	{
		// Получаем направление луча от камеры к курсору
		Vector3 direction = getRayDirection(cursorX, cursorY, imageWidth, imageHeight);

		// Коэффициент плавности поворота камеры (чем меньше, тем медленнее)
		float smoothingFactor = 0.1f;

		// Плавно изменяем направление камеры, чтобы она смотрела в сторону курсора
		forward = forward * (1 - smoothingFactor) + direction * smoothingFactor;

		// Нормализуем новое направление
		forward = forward.normalize();

		// Сохраняем вертикальное направление (up) неизменным
		right = forward.cross(up).normalize();  // Переопределяем направление right
	}
};

// Функция для вычисления преломленного луча (например, свет проходит через стекло)
Vector3 refract(const Vector3& I, const Vector3& N, float eta)
{
	// Вектор падения I, нормаль к поверхности N и коэффициент преломления eta

	float cosI = -I.dot(N);  // Косинус угла падения
	float sinT2 = eta * eta * (1 - cosI * cosI);  // Синус угла преломления (по закону Снеллиуса)

	// Если sinT2 больше 1, происходит полное внутреннее отражение, нет преломления
	if (sinT2 > 1) return Vector3(0, 0, 0);

	float cosT = std::sqrt(1 - sinT2);  // Косинус угла преломления
	// Возвращаем направление преломленного луча с учетом коэффициента преломления
	return I * eta + N * (eta * cosI - cosT);
}

// Трассировка луча: рекурсивная функция, вычисляющая цвет пикселя на основе пересечений луча с объектами сцены
Vector3 traceRay(const Vector3& origin, const Vector3& direction,
	const std::vector<Sphere>& spheres, const std::vector<Plane>& planes,
	const std::vector<Cube>& cubes, const std::vector<Light>& lights,
	const std::vector<Triangle>& triangles, int depth)
{
	// Если глубина рекурсии достигла нуля, возвращаем черный цвет (конец трассировки)
	if (depth <= 0) return Vector3(0, 0, 0);

	// Инициализируем переменные для минимальной дистанции до пересечения, точки пересечения, нормали и цвета
	float tMin = std::numeric_limits<float>::infinity(); // Изначально минимальное расстояние - бесконечность
	Vector3 hitPoint, normal, baseColor; // Точка пересечения, нормаль и основной цвет объекта
	float reflectivity = 0; // Коэффициент отражения
	float transmissivity = 0; // Коэффициент прозрачности
	float refractiveIndex = 1; // Показатель преломления (по умолчанию 1)

	// Проверка пересечений с различными объектами сцены (сферы, плоскости, кубы, треугольники)

	// Перебор всех сфер сцены
	for (const auto& sphere : spheres)
	{
		float t;
		// Проверка пересечения с каждой сферой
		if (sphere.intersect(origin, direction, t) && t < tMin)
		{
			// Если пересечение найдено, обновляем минимальное расстояние и свойства объекта
			tMin = t;
			hitPoint = origin + direction * t; // Вычисляем точку пересечения
			normal = (hitPoint - sphere.center).normalize(); // Нормаль к сфере в точке пересечения
			baseColor = sphere.color; // Основной цвет сферы
			reflectivity = sphere.reflectivity; // Отражение
			transmissivity = sphere.transmissivity; // Прозрачность
			refractiveIndex = sphere.refractiveIndex; // Показатель преломления
		}
	}

	// Перебор всех плоскостей сцены
	for (const auto& plane : planes)
	{
		float t;
		// Проверка пересечения с каждой плоскостью
		if (plane.intersect(origin, direction, t) && t < tMin)
		{
			// Если пересечение найдено, обновляем минимальное расстояние и свойства объекта
			tMin = t;
			hitPoint = origin + direction * t; // Вычисляем точку пересечения
			normal = plane.normal; // Нормаль плоскости
			baseColor = plane.color; // Основной цвет плоскости
			reflectivity = plane.reflectivity; // Отражение
		}
	}

	// Перебор всех кубов сцены
	for (const auto& cube : cubes)
	{
		float t;
		// Проверка пересечения с каждым кубом
		if (cube.intersect(origin, direction, t) && t < tMin)
		{
			// Если пересечение найдено, обновляем минимальное расстояние и свойства объекта
			tMin = t;
			hitPoint = origin + direction * t; // Вычисляем точку пересечения

			// Определение нормали к поверхности куба на основе координат точки пересечения
			if (std::abs(hitPoint.x - cube.min.x) < 1e-3) normal = Vector3(-1, 0, 0); // Левая грань
			else if (std::abs(hitPoint.x - cube.max.x) < 1e-3) normal = Vector3(1, 0, 0); // Правая грань
			else if (std::abs(hitPoint.y - cube.min.y) < 1e-3) normal = Vector3(0, -1, 0); // Нижняя грань
			else if (std::abs(hitPoint.y - cube.max.y) < 1e-3) normal = Vector3(0, 1, 0); // Верхняя грань
			else if (std::abs(hitPoint.z - cube.min.z) < 1e-3) normal = Vector3(0, 0, -1); // Задняя грань
			else if (std::abs(hitPoint.z - cube.max.z) < 1e-3) normal = Vector3(0, 0, 1); // Передняя грань

			baseColor = cube.color; // Основной цвет куба
			reflectivity = cube.reflectivity; // Отражение
			transmissivity = cube.transmissivity; // Прозрачность
			refractiveIndex = cube.refractiveIndex; // Показатель преломления
		}
	}

	// Перебор всех треугольников сцены
	for (const auto& triangle : triangles)
	{
		float t;
		// Проверка пересечения с каждым треугольником
		if (triangle.intersect(origin, direction, t) && t < tMin)
		{
			// Если пересечение найдено, обновляем минимальное расстояние и свойства объекта
			tMin = t;
			hitPoint = origin + direction * t; // Вычисляем точку пересечения
			normal = triangle.normal; // Нормаль треугольника
			baseColor = triangle.color; // Основной цвет треугольника
			reflectivity = triangle.reflectivity; // Отражение
			transmissivity = triangle.transmissivity; // Прозрачность
			refractiveIndex = triangle.refractiveIndex; // Показатель преломления
		}
	}

	// Если пересечений не найдено, возвращаем черный цвет
	if (tMin == std::numeric_limits<float>::infinity()) return Vector3(0, 0, 0);

	// Освещение: вычисляем вклад света для точки пересечения
	Vector3 color(0, 0, 0);
	for (const auto& light : lights)
	{
		// Направление света из источника в точку пересечения
		Vector3 lightDir = (light.position - hitPoint).normalize();
		// Освещенность в точке пересечения
		Vector3 lightColor = baseColor * std::max(0.f, normal.dot(lightDir));
		// Добавляем вклад света
		color = color + lightColor * light.intensity;
	}

	// Отражение: если объект отражающий, трассируем отраженный луч
	if (reflectivity > 0)
	{
		// Направление отраженного луча
		Vector3 reflectDir = direction - normal * 2 * direction.dot(normal);
		// Рекурсивно трассируем отраженный луч
		color = color + traceRay(hitPoint + normal * 1e-4, reflectDir, spheres, planes, cubes, lights, triangles, depth - 1) * reflectivity;
	}

	// Преломление: если объект прозрачный, трассируем преломленный луч
	if (transmissivity > 0)
	{
		// Вычисление отношения показателей преломления
		float eta = direction.dot(normal) < 0 ? 1 / refractiveIndex : refractiveIndex;
		// Направление преломленного луча
		Vector3 refractDir = refract(direction, normal, eta);
		// Рекурсивно трассируем преломленный луч
		color = color + traceRay(hitPoint - normal * 1e-4, refractDir, spheres, planes, cubes, lights, triangles, depth - 1) * transmissivity;
	}

	return color; // Возвращаем вычисленный цвет
}

// Мьютекс для защиты доступа к сцене
std::mutex sceneMutex;

// Функция для добавления нового куба в сцену
void addCube(std::vector<Cube>& cubes, const Camera& camera)
{
	std::lock_guard<std::mutex> lock(sceneMutex); // Защищаем критическую секцию
	// Создаем куб на расстоянии 3 единицы от камеры вдоль ее взгляда
	Vector3 position = camera.position + camera.forward * 5.0f; // Расстояние от камеры
	Cube newCube(position - Vector3(0.5f, 0.5f, 0.5f), position + Vector3(0.5f, 0.5f, 0.5f), Vector3(1, 0, 0), 0.2, 0.3, 1.0);
	cubes.push_back(newCube);
}

// Функция для добавления новой сферы в сцену
void addSphere(std::vector<Sphere>& spheres, const Camera& camera)
{
	std::lock_guard<std::mutex> lock(sceneMutex); // Защищаем критическую секцию
	// Создаем сферу на расстоянии 3 единицы от камеры вдоль ее взгляда
	Vector3 position = camera.position + camera.forward * 5.0f; // Расстояние от камеры
	Sphere newSphere(position, 0.7f, Vector3(0, 0, 1), 0.2f, 0.3f, 1.0f);
	spheres.push_back(newSphere);
}

// Функция для добавления нового источника света в сцену
void addLight(std::vector<Light>& lights, const Camera& camera)
{
	std::lock_guard<std::mutex> lock(sceneMutex); // Защищаем критическую секцию
	// Создаем новый источник света в точке, куда направлена камера
	Vector3 position = camera.position + camera.forward * 5.0f; // Расстояние от камеры
	Light newLight(position, Vector3(0.8, 0.8, 0.8)); // Белый свет
	lights.push_back(newLight);
}

// Функция для рендеринга строки изображения
void renderRow(int startY, int endY, sf::Image& image, const Camera& camera,
	const std::vector<Sphere>& spheres, const std::vector<Plane>& planes,
	const std::vector<Cube>& cubes, const std::vector<Light>& lights,
	const std::vector<Triangle>& triangles, int traceDepth, int quality)
{
#pragma omp parallel for
	for (int y = startY; y < endY; y += quality)
	{
		for (int x = 0; x < WIDTH; x += quality)
		{
			// Получаем направление луча из камеры для пикселя
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


// Функция для параллельного рендеринга изображения с оптимизацией
void renderImageParallelOptimized(
	sf::Image& image, const Camera& camera,
	const std::vector<Sphere>& spheres, const std::vector<Plane>& planes,
	const std::vector<Cube>& cubes, const std::vector<Light>& lights,
	const std::vector<Triangle>& triangles, // Добавлено
	int traceDepth, int WIDTH, int HEIGHT, int quality)
{
	// Создаем пул потоков
	int numThreads = std::thread::hardware_concurrency(); // Количество доступных потоков
	int rowsPerThread = HEIGHT / numThreads; // Количество строк, обрабатываемых каждым потоком

	std::vector<std::thread> threadPool; // Массив потоков

	// Распределяем рендеринг по потокам
	for (int i = 0; i < numThreads; ++i)
	{
		int startY = i * rowsPerThread; // Начало строки для потока
		int endY = (i == numThreads - 1) ? HEIGHT : (i + 1) * rowsPerThread; // Конец строки для потока

		// Запускаем поток для рендеринга строк
		threadPool.push_back(std::thread(renderRow, startY, endY, std::ref(image),
			std::cref(camera), std::cref(spheres), std::cref(planes),
			std::cref(cubes), std::cref(lights), std::cref(triangles), traceDepth, quality));
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
	// Устанавливаем количество потоков для OpenMP (параллельная обработка)
	omp_set_num_threads(36);

	// Настройки рендеринга: максимальная глубина трассировки и качество
	int traceDepth = 1; // Начальная глубина трассировки
	int quality = 1; // Начальное качество (1 - низкое качество)

	// Создаем окно с размером ширины WIDTH и высоты HEIGHT
	sf::RenderWindow window(sf::VideoMode(WIDTH, HEIGHT), "Optimized Scene Rendering");

	// Создаем изображение для рендеринга
	sf::Image image;
	image.create(WIDTH, HEIGHT);

	// Создаем пустые вектора для объектов на сцене
	std::vector<Sphere> spheres = {};  // Сферы
	std::vector<Plane> planes = {  // Плоскости (одна плоскость на земле)
		Plane(Vector3(0, -2, 0), Vector3(0, 1, 0), Vector3(0.8, 0.8, 0.8), 0.3),  // Плоскость с цветом и коэффициентом отражения
	};
	std::vector<Cube> cubes = {};  // Кубы
	std::vector<Light> lights = {
		// Добавляем источник света в сцену
		Light(Vector3(10, 10, -10), Vector3(0.8, 0.8, 0.8), Vector3(1, 0, 0)) // Прожектор
	};
	std::vector<Triangle> triangles; // Для треугольников из OBJ

	// Загрузка данных из OBJ файла (для треугольников)
	// Раскомментировать, если хотите загрузить 3D-модель из файла
	//std::string inputfile = "cube.obj";
	//std::string inputfile = "dodecahedron.obj";

	tinyobj::attrib_t attrib;
	std::vector<tinyobj::shape_t> shapes;
	std::vector<tinyobj::material_t> materials;
	std::string warn, err;

	// Загрузка OBJ файла
	//if (!tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, inputfile.c_str())) {
	//    std::cerr << "Ошибка загрузки OBJ: " << warn << err << std::endl;
	//    return 1;
	//}

	// Преобразование данных из OBJ в треугольники
	// Проходим по каждой форме в файле OBJ и создаем треугольники
	for (const auto& shape : shapes) {
		for (size_t i = 0; i < shape.mesh.indices.size(); i += 3) {
			tinyobj::index_t idx0 = shape.mesh.indices[i];
			tinyobj::index_t idx1 = shape.mesh.indices[i + 1];
			tinyobj::index_t idx2 = shape.mesh.indices[i + 2];

			// Получаем вершины треугольника
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
			triangles.emplace_back(v0, v1, v2, color, reflection); // Добавляем треугольник в список
		}
	}

	// Создаем камеру, которая будет отображать сцену
	Camera camera(Vector3(0, 2, -0.5), Vector3(-1, 2, -0.5), Vector3(0, 1, 0));

	// Рендеринг сцены с помощью многопоточности и трассировки лучей
	renderImageParallelOptimized(image, camera, spheres, planes, cubes, lights, triangles, traceDepth, WIDTH, HEIGHT, quality);

	// Создаем текстуру из изображения
	sf::Texture texture;
	texture.loadFromImage(image);
	sf::Sprite sprite(texture);

	// Полоска для отображения FPS
	sf::RectangleShape fpsBar(sf::Vector2f(200.f, 20.f));
	fpsBar.setPosition(10.f, 10.f);

	// Таймер для вычисления FPS
	sf::Clock clock;
	int RTturn = 0, Qturn = 0;

	// Главный цикл окна
	while (window.isOpen()) {
		sf::Event event;
		while (window.pollEvent(event)) {
			if (event.type == sf::Event::Closed) {
				window.close(); // Закрыть окно при закрытии
			}

			// Обработчик нажатий клавиш для добавления объектов на сцену
			if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Num1) {
				addCube(cubes, camera); // Добавляем куб
				renderImageParallelOptimized(image, camera, spheres, planes, cubes, lights, triangles, traceDepth, WIDTH, HEIGHT, quality);
				texture.loadFromImage(image); // Перерисовываем изображение
				sprite.setTexture(texture);   // Обновляем текстуру
				window.clear();
				window.draw(sprite);          // Отображаем новое изображение
				window.draw(fpsBar);          // Отображаем полоску FPS
				window.display();
			}

			if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Num2) {
				addSphere(spheres, camera); // Добавляем сферу
				renderImageParallelOptimized(image, camera, spheres, planes, cubes, lights, triangles, traceDepth, WIDTH, HEIGHT, quality);
				texture.loadFromImage(image);
				sprite.setTexture(texture);
				window.clear();
				window.draw(sprite);
				window.draw(fpsBar);
				window.display();
			}

			// Изменение глубины трассировки с помощью клавиши R
			if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::R) {
				RTturn++;
				traceDepth = (RTturn % 2 == 0) ? 1 : 4; // Чередуем глубину трассировки

				renderImageParallelOptimized(image, camera, spheres, planes, cubes, lights, triangles, traceDepth, WIDTH, HEIGHT, quality);
				texture.loadFromImage(image);
				sprite.setTexture(texture);
				window.clear();
				window.draw(sprite);
				window.draw(fpsBar);
				window.display();
			}

			// Добавление источника света с помощью клавиши L
			if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::L) {
				addLight(lights, camera); // Добавляем источник света
				renderImageParallelOptimized(image, camera, spheres, planes, cubes, lights, triangles, traceDepth, WIDTH, HEIGHT, quality);
				texture.loadFromImage(image);
				sprite.setTexture(texture);
				window.clear();
				window.draw(sprite);
				window.draw(fpsBar);
				window.display();
			}

			// Изменение качества рендеринга с помощью клавиши Q
			if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Q) {
				Qturn++;
				quality = (Qturn % 2 == 0) ? 1 : 2; // Чередуем качество

				image.create(WIDTH, HEIGHT, sf::Color(0, 0, 0)); // Перезапускаем изображение

				renderImageParallelOptimized(image, camera, spheres, planes, cubes, lights, triangles, traceDepth, WIDTH, HEIGHT, quality);
				texture.loadFromImage(image);
				sprite.setTexture(texture);
				window.clear();
				window.draw(sprite);
				window.draw(fpsBar);
				window.display();
			}

			// Обработка движения мыши для изменения угла обзора камеры
			if (event.type == sf::Event::MouseMoved) {
				camera.lookAtCursor(event.mouseMove.x, event.mouseMove.y, WIDTH, HEIGHT); // Изменяем направление взгляда камеры

				renderImageParallelOptimized(image, camera, spheres, planes, cubes, lights, triangles, traceDepth, WIDTH, HEIGHT, quality);
				texture.loadFromImage(image);
				sprite.setTexture(texture);
				window.clear();
				window.draw(sprite);
				window.draw(fpsBar);
				window.display();
			}
		}

		// Управление камерой с помощью клавиш W, S, A, D
		float cameraSpeed = 0.5f;
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::W)) {
			camera.position = camera.position + camera.forward * cameraSpeed; // Движение вперед
		}
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::S)) {
			camera.position = camera.position - camera.forward * cameraSpeed; // Движение назад
		}
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::A)) {
			camera.position = camera.position - camera.right * cameraSpeed; // Движение влево
		}
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::D)) {
			camera.position = camera.position + camera.right * cameraSpeed; // Движение вправо
		}

		// Вычисляем FPS
		float elapsedTime = clock.restart().asSeconds();
		float fps = 1.f / elapsedTime;

		// Нормализуем и отображаем полосу FPS
		float normalizedFps = std::min(fps / 60.f, 1.f);
		fpsBar.setSize(sf::Vector2f(200.f * normalizedFps, 20.f));

		// Изменяем цвет полосы в зависимости от FPS
		if (fps >= 50)
			fpsBar.setFillColor(sf::Color::Green);
		else if (fps >= 20)
			fpsBar.setFillColor(sf::Color::Yellow);
		else
			fpsBar.setFillColor(sf::Color::Red);

		// Рендерим изображение каждый кадр
		renderImageParallelOptimized(image, camera, spheres, planes, cubes, lights, triangles, traceDepth, WIDTH, HEIGHT, quality);
		texture.loadFromImage(image);
		sprite.setTexture(texture);

		window.clear();
		window.draw(sprite);
		window.draw(fpsBar);
		window.display();
	}

	return 0;
}
