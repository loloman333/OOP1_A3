//----------------------------------------------------------------------------------------------------------------------
// Player.cpp
//
// The Player class represents a Player on the board.
// It has several variables getter and setter for a player.
//
// Authors: Triochter Bande (Grill Matthias, Killer Lorenz, Nagy Lukas)
//----------------------------------------------------------------------------------------------------------------------
//

#include "Player.hpp"
#include "Treasure.hpp"

Player::Player(PlayerColor player_color) : nr_found_treasures_{0}, player_color_(player_color) {}

const std::vector<PlayerColor> Player::player_colors_ = {PlayerColor::RED, PlayerColor::YELLOW, PlayerColor::GREEN,
                                                         PlayerColor::BLUE};

std::vector<Treasure*>& Player::getCoveredStackRef()
{
  return covered_stack_;
}

size_t Player::getNrFoundTreasures()
{
  return nr_found_treasures_;
}

PlayerColor Player::getPlayerColor()
{
  return player_color_;
}

std::string Player::getPlayerColorAsString()
{
  switch (player_color_)
  {
    case PlayerColor::RED:
      return "Red";
    case PlayerColor::YELLOW:
      return "Yellow";
    case PlayerColor::GREEN:
      return "Green";
    case PlayerColor::BLUE:
      return "Blue";
  }
}

size_t Player::getCol()
{
  return col_;
}

size_t Player::getRow()
{
  return row_;
}

Item* Player::getItem()
{
  return item_;
}

bool Player::getUsingLadder()
{
  return using_ladder_;
}

bool Player::getTied()
{
  return tied_;
}

void Player::setNrFoundTreasures(size_t nr_found_treasures)
{
  nr_found_treasures_ = nr_found_treasures;
}

void Player::setCol(size_t col)
{
  col_ = col;
}

void Player::setRow(size_t row)
{
  row_ = row;
}

void Player::setRowCol(size_t row, size_t column)
{
  setRow(row);
  setCol(column);
}

void Player::setItem(Item* item)
{
  item_ = item;
}

void Player::setUsingLadder(bool using_ladder)
{
  using_ladder_ = using_ladder;
}

void Player::setTied(bool tied)
{
  tied_ = tied;
}

void Player::incrementNrFoundTreasures()
{
  nr_found_treasures_++;
}
