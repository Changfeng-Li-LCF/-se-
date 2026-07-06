#include "View.h"

void GameView::displayWelcome() const {
    std::cout << "==============================\n";
    std::cout << "       井字棋游戏\n";
    std::cout << "==============================\n";
}

void GameView::displayBoard(const GameModel& model) const {
    std::cout << "\n  1 2 3\n"; // 列号（用户视角1-3）
    for (int i = 0; i < 3; ++i) {
        std::cout << i + 1 << " "; // 行号（用户视角1-3）
        for (int j = 0; j < 3; ++j) {
            std::cout << model.getCell(i, j);
            if (j < 2) std::cout << "|";
        }
        std::cout << "\n";
        if (i < 2) std::cout << "  -+-+-\n";
    }
    std::cout << "\n";
}

void GameView::displayCurrentPlayer(char player) const {
    std::cout << "当前回合：玩家 " << player << "\n";
}

void GameView::displayPrompt() const {
    std::cout << "请输入行号和列号（1-3，空格分隔）：";
}

void GameView::displayInvalidInput() const {
    std::cout << "输入无效！请输入1-3范围内的坐标，且位置不能已有棋子。\n";
}

void GameView::displayWin(char player) const {
    std::cout << "游戏结束！玩家 " << player << " 获胜！\n";
}

void GameView::displayDraw() const {
    std::cout << "游戏结束，双方平局！\n";
}

void GameView::displayModeMenu() const {
    std::cout << "请选择游戏模式：\n";
    std::cout << "1. 双人对战\n";
    std::cout << "2. 人机对战（玩家X先手，AI为O）\n";
    std::cout << "请输入选项：";
}