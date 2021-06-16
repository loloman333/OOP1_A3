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
#include "Game.hpp"
#include "Player.hpp"
#include "Tile.hpp"

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

    // Phase 1: Add neighbors
    bool found_path = addNeighbors(current_tile, current_row, current_column, to_tile, discovered, unvisited);

    // Stop if neighbor is to_item
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
          Coordinates cords = std::make_pair<size_t, size_t>(new_row, new_column);
          TileAndCoordinates tile_info = std::pair<Tile*, Coordinates>{game_.getBoard()[new_row][new_column], cords};

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

void AIMaster::playAI()
{
  return;
}