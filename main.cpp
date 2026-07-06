#include "Model.h"
#include "View.h"
#include "Controller.h"
#include <iostream>

int main() {
    GameModel model;
    GameView view;
    GameController controller(model, view);


    // 模式选择
    view.displayModeMenu();
    int choice;
    while (true) {
        if (std::cin >> choice && (choice == 1 || choice == 2)) {
            break;
        }
        std::cin.clear();
        while (std::cin.get() != '\n');
        std::cout << "无效选项，请输入 1 或 2：";
    }

    // 启动对应模式
    if (choice == 1) {
        controller.runTwoPlayer();
    }
    else {
        controller.runAIPlayer();
    }

    std::cout << "\n按任意键退出...";
    std::cin.ignore();
    std::cin.get();
    return 0;
}