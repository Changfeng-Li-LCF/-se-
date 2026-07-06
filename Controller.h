#pragma once
#include "Model.h"
#include "View.h"

class GameController {
private:
    GameModel& model;
    GameView& view;

    // 读取玩家输入，转换为0-2的数组下标
    bool getPlayerInput(int& row, int& col);

public:
    GameController(GameModel& m, GameView& v);

    // 启动双人对战模式
    void runTwoPlayer();
    // 启动人机对战模式
    void runAIPlayer();
};
