//----------------------------------------------------------------------------------------------------------------------
// Tile.cpp
//
// Tile class represents a tile on the Board, is a super class for other tiles and can have walls and players on it
//
// Authors: Triochter Bande (Grill Matthias, Killer Lorenz, Nagy Lukas)
//----------------------------------------------------------------------------------------------------------------------
//

#include "Tile.hpp"
#include "Player.hpp"
#include "stdlib.h"

Tile::Tile(TileType type, Rotation rotation) : type_{type}, rotation_{rotation}
{
  generateWalls();
}
Tile::Tile(TileType type) : Tile(type, Rotation::DEG0) {}

const std::vector<Direction> Tile::all_directions_{Direction::TOP, Direction::LEFT, Direction::BOTTOM, Direction::RIGHT};

TileType Tile::getType()
{
  return type_;
}

Rotation Tile::getRotation()
{
  return rotation_;
}

std::vector<Player*> Tile::getPlayers()
{
  return players_;
}

void Tile::setRotation(Rotation rotation)
{
  Rotation old_rotation = rotation_;
  rotation_ = rotation;
  updateWallsAccordingToRotation(old_rotation);
}

void Tile::setType(TileType type)
{
  type_ = type;
  generateWalls();
}

size_t Tile::getRotationValue()
{
  switch (rotation_)
  {
    case Rotation::DEG0:
      return 0;
    case Rotation::DEG90:
      return 90;
    case Rotation::DEG180:
      return 180;
    case Rotation::DEG270:
      return 270;
  }
}

std::string Tile::getTileTypeString()
{
  switch (type_)
  {
    case TileType::T:
      return "T";
    case TileType::L:
      return "L";
    case TileType::I:
      return "I";
    case TileType::O:
      return "O";
    case TileType::U:
      return "U";
    case TileType::X:
      return "X";
    default:
      return nullptr;
  }
}

bool Tile::isCorner(size_t row, size_t col)
{
  if ((row == TOP_ROW || row == BOTTOM_ROW) && (col <= LEFT_COLUMN || col >= RIGHT_COLUMN))
  {
    return true;
  }
  return false;
}

bool Tile::isWallInDirection(Direction direction)
{
  for (size_t index = 0; index < walls_.size(); index++)
  {
    if (walls_[index] == direction)
    {
      return true;
    }
  }
  return false;
}

Direction Tile::calcDirection(Direction dir, Rotation rot)
{
  size_t dir_value = static_cast<size_t>(dir);
  size_t rot_value = static_cast<size_t>(rot);
  dir_value += rot_value;
  if (dir_value >= DIRECTION_AMOUNT)
  {
    dir_value -= DIRECTION_AMOUNT;
  }
  return static_cast<Direction>(dir_value);
}

std::vector<std::string> Tile::getRawTileString()
{
  std::vector<std::string> tile_vector;
  fillTileString(tile_vector);
  addPlayersToTile(tile_vector);

  return tile_vector;
}

void Tile::setWallsAccordingToTileType()
{
  std::vector<Direction> new_walls_{};

  switch (type_)
  {
    case TileType::T:
      new_walls_.push_back(Direction::TOP);
      break;
    case TileType::L:
      new_walls_.push_back(Direction::LEFT);
      new_walls_.push_back(Direction::BOTTOM);
      break;
    case TileType::I:
      new_walls_.push_back(Direction::LEFT);
      new_walls_.push_back(Direction::RIGHT);
      break;
    case TileType::O:
      new_walls_.push_back(Direction::LEFT);
      new_walls_.push_back(Direction::RIGHT);
      new_walls_.push_back(Direction::BOTTOM);
      new_walls_.push_back(Direction::TOP);
      break;
    case TileType::U:
      new_walls_.push_back(Direction::RIGHT);
      new_walls_.push_back(Direction::LEFT);
      new_walls_.push_back(Direction::BOTTOM);
      break;
    case TileType::X:
      break;
  }
  setWalls(new_walls_);
}

void Tile::updateWallsAccordingToRotation()
{
  updateWallsAccordingToRotation(Rotation::DEG0);
}

void Tile::updateWallsAccordingToRotation(Rotation old_rotation)
{
  std::vector<Direction> new_walls_{};

  int old_rotation_int = static_cast<int>(old_rotation);
  int new_rotation_int = static_cast<int>(rotation_);

  int rotation_change = abs(old_rotation_int - new_rotation_int);
  int step = rotation_change;

  if (old_rotation_int > new_rotation_int)
  {
    step *= -1;
  }

  for (Direction direction : walls_)
  {
    size_t new_direction = (static_cast<size_t>(direction) + step);

    new_direction += DIRECTION_AMOUNT;
    new_direction %= DIRECTION_AMOUNT;

    new_walls_.push_back(static_cast<Direction>(new_direction));
  }

  setWalls(new_walls_);
}

void Tile::setWalls(std::vector<Direction> directions)
{
  walls_ = directions;
}

void Tile::generateWalls()
{
  setWallsAccordingToTileType();
  updateWallsAccordingToRotation();
}

void Tile::fillTileString(std::vector<std::string>& tile)
{
  std::vector<std::string> template_tile;
  for (size_t row = 0; row < TILE_HEIGHT; row++)
  {
    template_tile.push_back("");
    for (size_t col = 0; col < TILE_WIDTH; col++)
    {
      if (isCorner(row, col))
      {
        template_tile[row].append(WALL);
      }
      else if (isWallInDirection(Direction::TOP) && row == TOP_ROW)
      {
        template_tile[row].append(WALL);
      }
      else if (isWallInDirection(Direction::LEFT) && col <= LEFT_COLUMN)
      {
        template_tile[row].append(WALL);
      }
      else if (isWallInDirection(Direction::BOTTOM) && row == BOTTOM_ROW)
      {
        template_tile[row].append(WALL);
      }
      else if (isWallInDirection(Direction::RIGHT) && col >= RIGHT_COLUMN)
      {
        template_tile[row].append(WALL);
      }
      else
      {
        template_tile[row].append(" ");
      }
    }
  }
  tile = template_tile;
}

void Tile::addPlayersToTile(std::vector<std::string>& tile_vector)
{
  if (!players_.empty())
  {

    std::string player_string = "P";

    for (PlayerColor color : Player::player_colors_)
    {
      if (isPlayerColorOnTile(color))
      {
        player_string.push_back(static_cast<char>(color));
      }
    }

    int position = STANDARD_POSITION;
    if (players_.size() == MAX_PLAYERS)
    {
      position = MAX_PLAYER_POSITION;
    }

    if (tile_vector[PLAYER_ROW][0] == ' ')
    {
      tile_vector[PLAYER_ROW].replace(position, player_string.size(), player_string);
    }
    else
    {
      tile_vector[PLAYER_ROW].replace(position + WALL_SIZE, player_string.size(), player_string);
    }
  }
}

bool Tile::isPlayerColorOnTile(PlayerColor color)
{
  for (Player* player : players_)
  {
    if (player->getPlayerColor() == color)
    {
      return true;
    }
  }
  return false;
}

void Tile::addPlayer(Player* player)
{
  players_.push_back(player);
}

void Tile::print()
{
  std::vector<std::string> tile_string = getTileString();
  for (std::string row : tile_string)
  {
    std::cout << row << std::endl;
  }
}

void Tile::removePlayer(std::string player_color)
{
  size_t location = 0;
  for (size_t index = 0; index < players_.size(); index++)
  {
    if (players_[index]->getPlayerColorAsString() == player_color)
    {
      location = index;
    }
  }
  players_.erase(players_.begin() + location);
}

void Tile::rotate(Direction dir)
{
  int step = 0;
  if (dir == Direction::LEFT)
  {
    step = 1;
  }
  else if (dir == Direction::RIGHT)
  {
    step = -1;
  }

  int new_rotation = (static_cast<size_t>(getRotation()) + step);
  new_rotation += ROTATION_AMOUNT;
  new_rotation %= ROTATION_AMOUNT;

  setRotation(static_cast<Rotation>(new_rotation));
}

bool Tile::hasTreasure()
{
  return false;
}

bool Tile::hasItem()
{
  return false;
}

Player* Tile::getPlayer(PlayerColor player_color)
{
  for (Player* player : players_)
  {
    if (player->getPlayerColor() == player_color)
    {
      return player;
    }
  }
  return nullptr;
}

bool Tile::addWallInDirection(Direction direction)
{
  if (isWallInDirection(direction))
  {
    return false;
  }
  else
  {
    walls_.push_back(direction);
    return true;
  }
}

void Tile::removeWallInDirection(Direction direction)
{
  size_t remove_index = walls_.size();
  for (size_t index = 0; index < walls_.size(); index++)
  {
    if (walls_[index] == direction)
    {
      remove_index = index;
    }
  }

  if (remove_index != walls_.size())
  {
    walls_.erase(walls_.begin() + remove_index);
  }
}
