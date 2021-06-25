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

bool AIMaster::playAI()
{
  Coordinates desired_coordinates = getDesiredCoordinates();

  std::vector<std::vector<std::string>> commands;

  if (! (game_.getCommandMaster()->getInserted()))
  {
    makeInsert(commands, desired_coordinates);
    desired_coordinates = getDesiredCoordinates();
  }

  playerGo(commands, desired_coordinates);

  return executeAllCommands(commands);
}

bool AIMaster::executeAllCommands(std::vector<std::vector<std::string>> &commands)
{
  std::vector<std::string> finish_command = {"finish"};

  for (std::vector<std::string> command : commands)
  {
    game_.getPrintMaster()->printAICommand(command);
  }
  game_.getPrintMaster()->printAICommand(finish_command);

  for (std::vector<std::string> command : commands)
  {
    game_.getCommandMaster()->executeCommand(command);
  }
  return game_.getCommandMaster()->executeCommand(finish_command);
}

void AIMaster::makeInsert(std::vector<std::vector<std::string>>& commands, Coordinates desired_coordinates)
{
  std::vector<std::string> directions = {"top", "left", "bottom", "right"};
  std::vector<std::string> indices = {"2", "4", "6"};
  
  for (std::string direction : directions)
  {
    for (std::string index : indices)
    {
      if (testInsert(direction, index, desired_coordinates) && checkLastInsert(direction, index))
      {
        std::vector<std::string> insert = {"insert", direction, index};         
        commands.push_back(insert);
        return;
      }
      undoFakeInsert(direction, index);
    }
  }

  randomInsert(commands);
}

void AIMaster::randomInsert(std::vector<std::vector<std::string>> &commands)
{
  std::vector<std::string> directions = {"top", "left", "bottom", "right"};
  std::vector<std::string> indices = {"2", "4", "6",};

  size_t random_direction_index;
  size_t random_index_index;

  do
  {
    random_direction_index = rand() % 4;
    random_index_index = rand() % 3;
  } while (!checkLastInsert(directions[random_direction_index], indices[random_index_index]));

  std::vector<std::string> insert = {"insert", directions[random_direction_index], indices[random_index_index]};

  if (directions[random_direction_index] == "left" || directions[random_direction_index] == "right")
  {
    fakeInsertRow(directions[random_direction_index], indices[random_index_index]);
  }
  else
  {
    fakeInsertColumn(directions[random_direction_index], indices[random_index_index]);
  }

  commands.push_back(insert);
}

bool AIMaster::testInsert(std::string direction, std::string index, Coordinates desired_coordinates)
{
  if (direction == "left" || direction == "right")
  {
    fakeInsertRow(direction, index);
  }
  else
  {
    fakeInsertColumn(direction, index);
  }

  if (isConnected(game_.getCurrentPlayer(), desired_coordinates.first + 1, desired_coordinates.second + 1))
  {
    return true;
  }

  return false;
}

void AIMaster::fakeInsertRow(std::string direction, std::string index)
{
  size_t row = std::stoi(index) - 1;
  size_t last_tile_index = BOARD_SIZE - 1;
  Tile* temp_free_tile = game_.getFreeTile();
  std::vector<std::vector<Tile*>>& board = game_.getBoard();
  if (direction == "l" || direction == "left")
  {
    game_.setFreeTile(board[row][last_tile_index]);
    for (size_t column = last_tile_index; column > 0; column--)
    {
      board[row][column] = board[row][column - 1];
      game_.getCommandMaster()->playersUpdateRowColumn(board[row][column]->getPlayers(), row, column);
    }
    board[row][0] = temp_free_tile;
    game_.getCommandMaster()->movePlayersToTile(game_.getFreeTile(), row, 0);
  }
  else
  {
    game_.setFreeTile(board[row][0]);
    for (size_t column = 0; column < last_tile_index; column++)
    {
      board[row][column] = board[row][column + 1];
      game_.getCommandMaster()->playersUpdateRowColumn(board[row][column]->getPlayers(), row, column);
    }
    board[row][BOARD_SIZE - 1] = temp_free_tile;
    game_.getCommandMaster()->movePlayersToTile(game_.getFreeTile(), row, BOARD_SIZE - 1);
  }
}

void AIMaster::fakeInsertColumn(std::string direction, std::string index)
{
  size_t column = std::stoi(index) - 1;
  size_t last_tile_index = BOARD_SIZE - 1;
  Tile* temp_free_tile = game_.getFreeTile();
  std::vector<std::vector<Tile*>>& board = game_.getBoard();

  if (direction == "t" || direction == "top")
  {
    game_.setFreeTile(board[last_tile_index][column]);
    for (size_t row = BOARD_SIZE - 1; row > 0; row--)
    {
      board[row][column] = board[row - 1][column];
      game_.getCommandMaster()->playersUpdateRowColumn(board[row][column]->getPlayers(), row, column);
    }
    board[0][column] = temp_free_tile;
    game_.getCommandMaster()->movePlayersToTile(game_.getFreeTile(), 0, column);
  }
  else
  {
    game_.setFreeTile(board[0][column]);
    for (size_t row = 0; row < BOARD_SIZE - 1; row++)
    {
      board[row][column] = board[row + 1][column];
      game_.getCommandMaster()->playersUpdateRowColumn(board[row][column]->getPlayers(), row, column);
    }
    board[BOARD_SIZE - 1][column] = temp_free_tile;
    game_.getCommandMaster()->movePlayersToTile(game_.getFreeTile(), BOARD_SIZE - 1, column);
  }
}

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

void AIMaster::playerGo(std::vector<std::vector<std::string>>& commands, Coordinates desired_coordinates)
{
  Player* current_player = game_.getCurrentPlayer();
  int to_row = desired_coordinates.first;
  int to_column = desired_coordinates.second;
  std::vector<std::string> command;
  bool faked_insert = false;
  std::string fake_insert_direction;
  std::string fake_insert_index;

  if (commands.size() == 1)
  {
    fake_insert_direction = commands[0][1];
    fake_insert_index = commands[0][2];
    faked_insert = true;
  }

  int current_row = current_player->getRow();
  int current_column = current_player->getCol();  

  if ((to_row == -1 && to_column == -1) || (current_row == to_row && current_column == to_column))
  {
    if (faked_insert)
    {
      undoFakeInsert(fake_insert_direction, fake_insert_index);
    }
    return;
  }

  if (isConnected(current_player, to_row + 1, to_column + 1))
  {
    command.push_back("go");
    command.push_back(std::to_string(to_row + 1));
    command.push_back(std::to_string(to_column + 1));
    commands.push_back(command);
  }
  else
  {
    // TODO: Not connected - move as close as possible
  }

  if (faked_insert)
  {
    undoFakeInsert(fake_insert_direction, fake_insert_index);
  }
}

void AIMaster::undoFakeInsert(std::string direction, std::string index)
{
  if (direction == "left" || direction == "right")
  {
    fakeInsertRow(getOppositeDirection(direction), index);
  }
  else
  {
    fakeInsertColumn(getOppositeDirection(direction), index);
  }
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
    TileAndCoordinates{board[BOARD_SIZE - 1][0], Coordinates{6, 0}},
    TileAndCoordinates{board[0][BOARD_SIZE - 1], Coordinates{0, BOARD_SIZE - 1}},
    TileAndCoordinates{board[6][BOARD_SIZE - 1], Coordinates{6, BOARD_SIZE - 1}}
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

std::string AIMaster::getOppositeDirection(std::string direction)
{
  if(direction == "left")
  {
    return "right";
  }
  else if(direction == "right")
  {
    return "left";
  }
  else if(direction == "top")
  {
    return "bottom";
  }
  else
  {
    return "top";
  }
}

bool AIMaster::checkLastInsert(std::string direction, std::string index)
{
  if (game_.getCommandMaster()->last_insert_row_col_ == static_cast<size_t>(std::stoi(index)))
  {
    if (game_.getCommandMaster()->last_insert_direction_ == getOppositeDirection(direction))
    {
      return false;
    }
  }
  return true;
}

