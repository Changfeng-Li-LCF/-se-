#include "Controller.h"
#include <iostream>

GameController::GameController(GameModel& m, GameView& v) : model(m), view(v) {}

bool GameController::getPlayerInput(int& row, int& col) {
    int r, c;
    if (!(std::cin >> r >> c)) {
        // 处理非数字输入：清空错误标志与缓冲区
        std::cin.clear();
        while (std::cin.get() != '\n');
        return false;
    }
    // 用户输入1-3 → 数组下标0-2
    row = r - 1;
    col = c - 1;
    return true;
}

void GameController::runTwoPlayer() {
    model.reset();
    view.displayWelcome();

    while (true) {
        view.displayBoard(model);
        view.displayCurrentPlayer(model.getCurrentPlayer());
        view.displayPrompt();

        int row, col;
        // 输入非法或落子失败则重试
        if (!getPlayerInput(row, col) || !model.placePiece(row, col)) {
            view.displayInvalidInput();
            continue;
        }

        // 判定胜负
        if (model.checkWin()) {
            view.displayBoard(model);
            view.displayWin(model.getCurrentPlayer());
            break;
        }
        // 判定平局
        if (model.checkDraw()) {
            view.displayBoard(model);
            view.displayDraw();
            break;
        }

        // 切换回合
        model.switchPlayer();
    }
}

void GameController::runAIPlayer() {
    model.reset();
    view.displayWelcome();
    std::cout << "人机模式：你是玩家 X，AI 为玩家 O\n";

    while (true) {
        view.displayBoard(model);

        if (model.getCurrentPlayer() == 'X') {
            // 玩家回合
            view.displayCurrentPlayer('X');
            view.displayPrompt();

            int row, col;
            if (!getPlayerInput(row, col) || !model.placePiece(row, col)) {
                view.displayInvalidInput();
                continue;
            }
        }
        else {
            // AI 回合
            view.displayCurrentPlayer('O');
            std::cout << "AI 正在落子...\n";
            int row, col;
            model.getAIMove(row, col);
            model.placePiece(row, col);
        }

        // 判定胜负
        if (model.checkWin()) {
            view.displayBoard(model);
            view.displayWin(model.getCurrentPlayer());
            break;
        }
        // 判定平局
        if (model.checkDraw()) {
            view.displayBoard(model);
            view.displayDraw();
            break;
        }

        model.switchPlayer();
    }
}