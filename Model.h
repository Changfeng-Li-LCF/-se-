#pragma once

class GameModel {
private:
    char board[3][3];   // 3x3 棋盘，空格为 ' '，棋子为 'X' / 'O'
    char currentPlayer; // 当前回合玩家

public:
    GameModel();

    // 重置游戏状态
    void reset();
    // 落子：参数为数组下标(0-2)，返回是否落子成功
    bool placePiece(int row, int col);
    // 判断当前玩家是否获胜
    bool checkWin() const;
    // 判断是否平局
    bool checkDraw() const;
    // 切换当前玩家
    void switchPlayer();
    // 获取当前玩家
    char getCurrentPlayer() const;
    // 获取指定位置的棋子（供View层渲染）
    char getCell(int row, int col) const;
    // 简单AI：返回第一个可落子的空位
    void getAIMove(int& row, int& col) const;
};