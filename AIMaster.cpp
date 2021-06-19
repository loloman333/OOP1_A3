//----------------------------------------------------------------------------------------------------------------------
// AIMaster.cpp
//
// TODO
//
// Authors: Triochter Bande (Grill Matthias, Killer Lorenz, Nagy Lukas)
//----------------------------------------------------------------------------------------------------------------------
//

#include "AIMaster.hpp"
#include "CommandMaster.hpp"
#include "PrintMaster.hpp"
#include "Game.hpp"
#include "Player.hpp"
#include "Tile.hpp"
#include "Treasure.hpp"
#include "TreasureTile.hpp"
#include "StartTile.hpp"
#include <vector>

AIMaster::AIMaster(Game& game) : game_{game} {};

bool AIMaster::isConnected(Player* current_player, size_t to_row, size_t to_column)
{
  to_row -= 1;
  to_column -= 1;

  Tile* to_tile = game_.getBoard()[to_row][to_column];
  size_t current_row = current_player->getRow();
  size_t current_column = current_player->getCol();
  Tile* current_tile = game_.getBoard()[current_row][current_column];
  
  std::set<Tile*> discovered{current_tile};
  std::map<Tile*, Coordinates> unvisited{};

  if (current_tile == to_tile)
  {
    return true;
  } 

  while (current_tile != nullptr)
  {

    bool found_path = addNeighbors(current_tile, current_row, current_column, to_tile, discovered, unvisited);

    if (found_path)
    {
      return true;
    }

    current_tile = chooseNextTile(to_row, to_column, unvisited, current_row, current_column);
  }

  return false;
}

Tile* AIMaster::chooseNextTile(size_t to_row, size_t to_column, std::map<Tile*, Coordinates>& 
    unvisited, size_t& current_row, size_t& current_column)
{
  Tile* next_tile = nullptr;
  int min_distance = BOARD_SIZE * 2; 
  for (TileAndCoordinates entry : unvisited)
  { 
    int row_distance = abs(static_cast<int>(entry.second.first) - static_cast<int>(to_row));
    int col_distance = abs(static_cast<int>(entry.second.second) - static_cast<int>(to_column));
    int distance = col_distance + row_distance;

    if (distance < min_distance)
    {
      min_distance = distance;
      next_tile = entry.first;
      current_row = entry.second.first;
      current_column = entry.second.second;
    }
  }

  unvisited.erase(next_tile);
  return next_tile;
}

bool AIMaster::addNeighbors(Tile* current_tile, size_t current_row, size_t current_column, Tile* to_tile, 
  std::set<Tile*>& discovered, std::map<Tile*, Coordinates>& unvisited)
{
  std::vector<TileAndCoordinates> neighbors = getReachableNeighborsOfTile(current_tile, current_row, current_column);
  for (TileAndCoordinates neighbor : neighbors)
  {
    if (neighbor.first == to_tile)
    {
      return true;
    } 

    bool inserted = discovered.insert(neighbor.first).second;
    if (inserted)
    {
      unvisited.insert(neighbor);
    } 
  }

  return false;
}

std::vector<TileAndCoordinates> AIMaster::getReachableNeighborsOfTile(Tile* tile, size_t row, size_t column)
{
  std::vector<TileAndCoordinates> neighbors;

  for (Direction direction : Tile::all_directions_)
  {
    if (!(tile->isWallInDirection(direction))){

      int new_row = row + calculateRowChangeInDirection(direction);
      int new_column = column + calculateColumnChangeInDirection(direction);

      int board_size = static_cast<int>(BOARD_SIZE);
      if (new_row >= 0 && new_row < board_size && new_column >= 0 && new_column < board_size)
      {  
        Tile* neighbor = game_.getBoard()[new_row][new_column];
        if (!(neighbor->isWallInDirection(game_.getCommandMaster()->getOppositeDirection(direction))))
        {
          Coordinates cords{new_row, new_column};
          TileAndCoordinates tile_info{game_.getBoard()[new_row][new_column], cords};

          neighbors.push_back(tile_info);
        }
      }
    }
  }

  return neighbors;
}


int AIMaster::calculateRowChangeInDirection(Direction direction)
{
  switch (direction)
  {
    case Direction::TOP:
      return -1;
    case Direction::LEFT:
      return 0;
    case Direction::BOTTOM:
      return 1;
    case Direction::RIGHT:
      return 0;
    
    default:
      return 0;
  }
}

int AIMaster::calculateColumnChangeInDirection(Direction direction)
{
  switch (direction)
  {
    case Direction::TOP:
      return 0;
    case Direction::LEFT:
      return -1;
    case Direction::BOTTOM:
      return 0;
    case Direction::RIGHT:
      return 1;
    
    default:
      return 0;
  }
}

void AIMaster::playerGo(Coordinates desired_coordinates)
{
  Player* current_player = game_.getCurrentPlayer();
  int to_row = desired_coordinates.first;
  int to_column = desired_coordinates.second;
  std::vector<std::string> command;

  if (to_row == -1 && to_column == -1)
  {
    return; // TODO: maybe move somewhere?
  }

  command.push_back("go");
  command.push_back(std::to_string(to_row + 1));
  command.push_back(std::to_string(to_column + 1));
  game_.getPrintMaster()->printAICommand(command);
  game_.getCommandMaster()->executeCommand(command);

  int current_row = current_player->getRow();
  int current_column = current_player->getCol();

  if (current_row == to_row && current_column == to_column)
  {
    return;
  }

  // TODO: Move close to desired_coordinates?
}

Coordinates AIMaster::getTreasureCoordinates(Treasure *treasure)
{
  for (int row = 0; row < static_cast<int>(BOARD_SIZE); row++)
  {
    for (int column = 0; column < static_cast<int>(BOARD_SIZE); column++)
    {
      if (game_.getBoard()[row][column]->hasTreasure())
      {
        TreasureTile* treasure_tile = dynamic_cast<TreasureTile*>(game_.getBoard()[row][column]);
        if (treasure_tile->getTreasure() == treasure)
        {
          return Coordinates{row, column};
        }
      }
    }
  }
  return Coordinates{-1, -1};
}

Coordinates AIMaster::getHomeBaseCoordinates()
{
  Player* current_player = game_.getCurrentPlayer();

  std::vector<std::vector<Tile*>> board = game_.getBoard();
  std::vector<TileAndCoordinates> bases = 
  {
    TileAndCoordinates{board[0][0], Coordinates{0, 0}},
    TileAndCoordinates{board[6][0], Coordinates{6, 0}}, 
    TileAndCoordinates{board[0][6], Coordinates{0, 6}}, 
    TileAndCoordinates{board[6][6], Coordinates{6, 6}}
  };

  for (TileAndCoordinates base_info : bases)
  {
    if (dynamic_cast<StartTile*>(base_info.first)->getPlayerColor() == current_player->getPlayerColor())
    {
      return base_info.second;
    }
  }

  return Coordinates{-1, -1};
}

Coordinates AIMaster::getDesiredCoordinates()
{
  std::vector<Treasure*> treasures = game_.getCurrentPlayer()->getCoveredStackRef();
  if (treasures.empty())
  {
    return getHomeBaseCoordinates();
  }
  else
  {
    Treasure* current_treasure = treasures.back();
    return getTreasureCoordinates(current_treasure);
  }
}

bool AIMaster::playAI()
{
  Coordinates desired_coordinates = getDesiredCoordinates();

  if (! (game_.getCommandMaster()->getInserted()))
  {
    //TODO: makeInsert(desired_coordinates);
    desired_coordinates = getDesiredCoordinates();
  }

  playerGo(desired_coordinates);

  std::vector<std::string> command = {"finish"};
  game_.getPrintMaster()->printAICommand(command);
  return game_.getCommandMaster()->executeCommand(command);
}