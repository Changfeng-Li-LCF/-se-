#pragma once
#include "Model.h"
#include <iostream>

class GameView {
public:
    // 显示欢迎信息
    void displayWelcome() const;
    // 渲染当前棋盘
    void displayBoard(const GameModel& model) const;
    // 显示当前回合玩家
    void displayCurrentPlayer(char player) const;
    // 显示输入提示
    void displayPrompt() const;
    // 显示输入错误提示
    void displayInvalidInput() const;
    // 显示获胜结果
    void displayWin(char player) const;
    // 显示平局结果
    void displayDraw() const;
    // 显示模式选择菜单
    void displayModeMenu() const;
};