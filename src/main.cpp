#include <iostream>
#include <string>
#include <QApplication>
#include <mainwindow.h>


using namespace std;

int main(int argc, char* argv[]) {

    //mostro il form
	QApplication app(argc, argv);
	MainWindow* window = new MainWindow();
	window->show();
	return app.exec();
}
