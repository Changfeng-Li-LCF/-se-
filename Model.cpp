#include "Model.h"

GameModel::GameModel() {
    reset();
}

void GameModel::reset() {
    // 初始化棋盘为空
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            board[i][j] = ' ';
        }
    }
    currentPlayer = 'X'; // X 方先手
}

bool GameModel::placePiece(int row, int col) {
    // 校验坐标范围与位置是否为空
    if (row < 0 || row >= 3 || col < 0 || col >= 3) {
        return false;
    }
    if (board[row][col] != ' ') {
        return false;
    }
    board[row][col] = currentPlayer;
    return true;
}

bool GameModel::checkWin() const {
    // 检查横向三连
    for (int i = 0; i < 3; ++i) {
        if (board[i][0] == currentPlayer
            && board[i][1] == currentPlayer
            && board[i][2] == currentPlayer) {
            return true;
        }
    }
    // 检查纵向三连
    for (int j = 0; j < 3; ++j) {
        if (board[0][j] == currentPlayer
            && board[1][j] == currentPlayer
            && board[2][j] == currentPlayer) {
            return true;
        }
    }
    // 检查对角线三连
    if (board[0][0] == currentPlayer
        && board[1][1] == currentPlayer
        && board[2][2] == currentPlayer) {
        return true;
    }
    if (board[0][2] == currentPlayer
        && board[1][1] == currentPlayer
        && board[2][0] == currentPlayer) {
        return true;
    }
    return false;
}

bool GameModel::checkDraw() const {
    // 已分出胜负则不是平局
    if (checkWin()) {
        return false;
    }
    // 棋盘未满则不是平局
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            if (board[i][j] == ' ') {
                return false;
            }
        }
    }
    return true;
}

void GameModel::switchPlayer() {
    currentPlayer = (currentPlayer == 'X') ? 'O' : 'X';
}

char GameModel::getCurrentPlayer() const {
    return currentPlayer;
}

char GameModel::getCell(int row, int col) const {
    return board[row][col];
}

void GameModel::getAIMove(int& row, int& col) const {
    // 简单策略：按行优先遍历，选择第一个空位
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            if (board[i][j] == ' ') {
                row = i;
                col = j;
                return;
            }
        }
    }
}