//函数包装器
//同一不同函数调用方式
//保护成员函数接口

#include <iostream>
#include <string>
#include <functional>

// 自由函数
void save_with_free_fun(const std::string& file_name)
{
	std::cout << "调用了自由函数，保存：" << file_name << std::endl;
}

// 成员函数
class FileSave
{
public:
	FileSave() = default;
	~FileSave() = default;

	void save_with_member_fun(const std::string& file_name)
	{
		std::cout << "调用了成员函数，保存：" << file_name << std::endl;
	}
};

int main()
{
	// Lambda函数
	auto save_with_lambda_fun = [](const std::string& file_name) -> void
		{
			std::cout << "调用了Lambda函数，保存：" << file_name << std::endl;
		};

	// save_with_free_fun("file.txt");
	// file_name.save_with_member_fun("file.txt");
	// save_with_lambda_fun("file.txt");

	std::function<void(const std::string&)> save1 = save_with_free_fun;
	std::function<void(const std::string&)> save3 = save_with_lambda_fun;
	// 成员函数，放入包装器
	// 绑定（类的成员函数的指针，对象的指针，占位符）
	FileSave file_save;
	std::function<void(const std::string&)> save2 =
		std::bind(&FileSave::save_with_member_fun, &file_save, std::placeholders::_1);

	save1("file.txt");
	save2("file.txt");
	save3("file.txt");

	return 0;
}