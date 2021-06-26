//----------------------------------------------------------------------------------------------------------------------
// AIMaster.cpp
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

AIMaster::AIMaster(Game &game) : game_{game}
{};

bool AIMaster::playAI()
{
  Coordinates desired_coordinates = getDesiredCoordinates();

  std::vector<std::vector<std::string>> commands;

  if (!(game_.getCommandMaster()->getInserted()))
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

void AIMaster::makeInsert(std::vector<std::vector<std::string>> &commands, Coordinates &desired_coordinates)
{
  std::vector<std::string> directions = {"top", "left", "bottom", "right"};
  std::vector<std::string> indices = {"2", "4", "6"};
  size_t min_walls = -1;

  std::string best_direction;
  std::string best_index;
  size_t best_rotation;

  for (std::string direction : directions)
  {
    for (std::string index : indices)
    {
      if (checkLastInsert(direction, index))
      {
        for (size_t rotation = 0; rotation < 4; rotation++)
        {
          if (rotation != 0)
          {
            game_.getFreeTile()->rotate(Direction::LEFT);
          }
          size_t wall_count = testInsert(direction, index, desired_coordinates);
          if (wall_count < min_walls)
          {
            min_walls = wall_count;

            best_direction = direction;
            best_index = index;
            best_rotation = rotation;
          }
          undoFakeInsert(direction, index);
          if (rotation == 3)
          {
            game_.getFreeTile()->rotate(Direction::LEFT);
          }
        }
      }
    }
  }


  addRotation(commands, best_rotation);
  std::vector<std::string> insert = {"insert", best_direction, best_index};
  commands.push_back(insert);

  while (best_rotation != 0)
  {
    game_.getFreeTile()->rotate(Direction::LEFT);
    best_rotation--;
  }
  testInsert(best_direction, best_index, desired_coordinates);
  std::cout << min_walls << std::endl;
}

void AIMaster::addRotation(std::vector<std::vector<std::string>> &commands, size_t rotation)
{
  std::vector<std::string> rotation_command_left = {"rotate", "left"};
  std::vector<std::string> rotation_command_right = {"rotate", "right"};
  switch (rotation)
  {
    case 1:
      commands.push_back(rotation_command_left);
      break;
    case 2:
      commands.push_back(rotation_command_left);
      commands.push_back(rotation_command_left);
      break;
    case 3:
      commands.push_back(rotation_command_right);
      break;
    default:
      return;
  }
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
  } else
  {
    fakeInsertColumn(directions[random_direction_index], indices[random_index_index]);
  }

  commands.push_back(insert);
}

size_t AIMaster::testInsert(std::string direction, std::string index, Coordinates &desired_coordinates)
{
  if (direction == "left" || direction == "right")
  {
    fakeInsertRow(direction, index);
  } else
  {
    fakeInsertColumn(direction, index);
  }

  desired_coordinates = getDesiredCoordinates();

  if (desired_coordinates.first == -1 && desired_coordinates.second == -1)
  {
    return -1;
  }

  return getWallsToTile(game_.getCurrentPlayer(), desired_coordinates.first + 1, desired_coordinates.second + 1);
}

void AIMaster::fakeInsertRow(std::string direction, std::string index)
{
  size_t row = std::stoi(index) - 1;
  size_t last_tile_index = BOARD_SIZE - 1;
  Tile *temp_free_tile = game_.getFreeTile();
  std::vector<std::vector<Tile *>> &board = game_.getBoard();
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
  } else
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
  Tile *temp_free_tile = game_.getFreeTile();
  std::vector<std::vector<Tile *>> &board = game_.getBoard();

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
  } else
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

size_t AIMaster::getWallsToTile(Player *current_player, size_t to_row, size_t to_column)
{
  to_row -= 1;
  to_column -= 1;

  Tile *to_tile = game_.getBoard()[to_row][to_column];
  size_t current_row = current_player->getRow();
  size_t current_column = current_player->getCol();
  Tile *current_tile = game_.getBoard()[current_row][current_column];

  size_t wall_step = 0;

  TileInfoMap discovered{ std::pair<Tile*, CoordinatesAndWalls>{current_tile, CoordinatesAndWalls{Coordinates{current_row, current_column}, 0}}};
  TileInfoMap unvisited{};

  if (current_tile == to_tile)
  {
    return 0;
  }

  while (current_tile != nullptr)
  {
    addNeighbors(current_tile, discovered, unvisited);

    if (discovered.find(to_tile) != discovered.end() && wall_step > discovered[to_tile].second)
    {
      return discovered[to_tile].second;
    }

    current_tile = chooseNextTile(to_row, to_column, unvisited, current_row, current_column, wall_step);
  }

  return discovered[to_tile].second;
}

Tile *AIMaster::chooseNextTile(size_t to_row, size_t to_column, TileInfoMap& unvisited, size_t &current_row, size_t &current_column, size_t& wall_step)
{
  Tile *next_tile = nullptr;

  if (unvisited.empty())
  {
    return next_tile;
  }

  for (std::pair<Tile*, CoordinatesAndWalls> entry : unvisited)
  {
    if (entry.second.second == wall_step)
    {
      next_tile = entry.first;
      current_row = entry.second.first.first;
      current_column = entry.second.first.second;
    }
  }

  unvisited.erase(next_tile);

  if (next_tile == nullptr)
  {
    return chooseNextTile(to_row, to_column, unvisited, current_row, current_column, ++wall_step);
  }

  return next_tile;
}

void AIMaster::addNeighbors(Tile *current_tile, TileInfoMap &discovered, TileInfoMap &unvisited)
{
  std::vector<TileCoordinatesAndWalls> neighbors = getReachableNeighborsOfTile(current_tile, discovered[current_tile]);

  for (TileCoordinatesAndWalls neighbor : neighbors)
  {
    Tile* neighbor_pointer = std::get<0>(neighbor);
    Coordinates neighbor_coordinates= std::get<1>(neighbor);
    size_t neighbor_walls = std::get<2>(neighbor);

    if (discovered.find(neighbor_pointer) == discovered.end())
    {
      discovered.insert(std::pair<Tile*, CoordinatesAndWalls>{neighbor_pointer, CoordinatesAndWalls{neighbor_coordinates, neighbor_walls}});
      unvisited.insert(std::pair<Tile*, CoordinatesAndWalls>{neighbor_pointer, CoordinatesAndWalls{neighbor_coordinates, neighbor_walls}});
    }

    if (neighbor_walls < discovered[neighbor_pointer].second)
    {
      discovered[neighbor_pointer].second = neighbor_walls;
      unvisited.erase(neighbor_pointer);
      unvisited.insert(std::pair<Tile*, CoordinatesAndWalls>{neighbor_pointer, CoordinatesAndWalls{neighbor_coordinates, neighbor_walls}});
    }
  }

}

std::vector<TileCoordinatesAndWalls> AIMaster::getReachableNeighborsOfTile(Tile* tile, CoordinatesAndWalls info)
{
  std::vector<TileCoordinatesAndWalls> neighbors;

  for (Direction direction : Tile::all_directions_)
  {
    size_t base_wall_increase = 0;

    if (tile->isWallInDirection(direction))
    {
      base_wall_increase++;
    }

    int new_row = info.first.first + calculateRowChangeInDirection(direction);
    int new_column = info.first.second + calculateColumnChangeInDirection(direction);

    int board_size = static_cast<int>(BOARD_SIZE);
    if (new_row >= 0 && new_row < board_size && new_column >= 0 && new_column < board_size)
    {
      Tile *neighbor = game_.getBoard()[new_row][new_column];

      Coordinates cords{new_row, new_column};

      size_t special_wall_increase = 0;

      if ((neighbor->isWallInDirection(game_.getCommandMaster()->getOppositeDirection(direction))))
      {
        special_wall_increase++;
      }

      TileCoordinatesAndWalls tile_info{game_.getBoard()[new_row][new_column], cords, info.second + base_wall_increase + special_wall_increase};

      neighbors.push_back(tile_info);
    }
  }
  return neighbors;
}

std::vector<TileAndCoordinates> AIMaster::getNeighborsOfTile(size_t row, size_t column)
{
  std::vector<TileAndCoordinates> neighbors;

  for (Direction direction : Tile::all_directions_)
  {
    int new_row = row + calculateRowChangeInDirection(direction);
    int new_column = column + calculateColumnChangeInDirection(direction);

    int board_size = static_cast<int>(BOARD_SIZE);
    if (new_row >= 0 && new_row < board_size && new_column >= 0 && new_column < board_size)
    {
      Coordinates cords{new_row, new_column};
      TileAndCoordinates tile_info{game_.getBoard()[new_row][new_column], cords};

      neighbors.push_back(tile_info);
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

void AIMaster::playerGo(std::vector<std::vector<std::string>> &commands, Coordinates desired_coordinates)
{
  Player *current_player = game_.getCurrentPlayer();
  int to_row = desired_coordinates.first;
  int to_column = desired_coordinates.second;
  std::vector<std::string> command;
  bool faked_insert = false;
  std::string fake_insert_direction;
  std::string fake_insert_index;
  size_t left_counter = 0;
  size_t right_counter = 0;

  if (commands.size() > 0)
  {
    for (auto command : commands)
    {
      if (command[0] == "insert")
      {
        fake_insert_direction = command[1];
        fake_insert_index = command[2];
        faked_insert = true;
      } else
      {
        if (command[1] == "left")
        {
          left_counter++;
        } else
        {
          right_counter++;
        }
      }
    }
  }

  int current_row = current_player->getRow();
  int current_column = current_player->getCol();

  if ((to_row == -1 && to_column == -1) || (current_row == to_row && current_column == to_column))
  {
    if (faked_insert)
    {
      undoFakeInsert(fake_insert_direction, fake_insert_index);
      undoRotation(left_counter, right_counter);
    }
    return;
  }

  TileAndCoordinates current_tile{game_.getBoard()[to_row][to_column], Coordinates{to_row, to_column}};
  std::set<TileAndCoordinates> try_tiles;
  std::set<TileAndCoordinates> known_tiles;

  do
  {
    if (getWallsToTile(current_player, current_tile.second.first + 1, current_tile.second.second + 1) == 0)
    {
      command.push_back("go");
      command.push_back(std::to_string(current_tile.second.first + 1));
      command.push_back(std::to_string(current_tile.second.second + 1));
      commands.push_back(command);
      break;
    } else
    {
      std::vector<TileAndCoordinates> neighbors = getNeighborsOfTile(current_tile.second.first,
                                                                     current_tile.second.second);

      for (TileAndCoordinates neighbor : neighbors)
      {
        if (known_tiles.insert(neighbor).second)
        {
          try_tiles.insert(neighbor);
        }
      }
    }

    current_tile = *(try_tiles.begin());
    try_tiles.erase(try_tiles.begin());
  } while (try_tiles.size() != 0);

  if (faked_insert)
  {
    undoFakeInsert(fake_insert_direction, fake_insert_index);
    undoRotation(left_counter, right_counter);
  }
}

void AIMaster::undoRotation(size_t left_counter, size_t right_counter)
{
  while (left_counter != 0)
  {
    game_.getFreeTile()->rotate(Direction::RIGHT);
    left_counter--;
  }

  while (right_counter != 0)
  {
    game_.getFreeTile()->rotate(Direction::LEFT);
    right_counter--;
  }
}

void AIMaster::undoFakeInsert(std::string direction, std::string index)
{
  if (direction == "left" || direction == "right")
  {
    fakeInsertRow(getOppositeDirection(direction), index);
  } else
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
        TreasureTile *treasure_tile = dynamic_cast<TreasureTile *>(game_.getBoard()[row][column]);
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
  Player *current_player = game_.getCurrentPlayer();

  std::vector<std::vector<Tile *>> board = game_.getBoard();
  std::vector<TileAndCoordinates> bases =
    {
      TileAndCoordinates{board[0][0], Coordinates{0, 0}},
      TileAndCoordinates{board[BOARD_SIZE - 1][0], Coordinates{6, 0}},
      TileAndCoordinates{board[0][BOARD_SIZE - 1], Coordinates{0, BOARD_SIZE - 1}},
      TileAndCoordinates{board[6][BOARD_SIZE - 1], Coordinates{6, BOARD_SIZE - 1}}
    };

  for (TileAndCoordinates base_info : bases)
  {
    if (dynamic_cast<StartTile *>(base_info.first)->getPlayerColor() == current_player->getPlayerColor())
    {
      return base_info.second;
    }
  }

  return Coordinates{-1, -1};
}

Coordinates AIMaster::getDesiredCoordinates()
{
  std::vector<Treasure *> treasures = game_.getCurrentPlayer()->getCoveredStackRef();
  if (treasures.empty())
  {
    return getHomeBaseCoordinates();
  } else
  {
    Treasure *current_treasure = treasures.back();
    return getTreasureCoordinates(current_treasure);
  }
}

std::string AIMaster::getOppositeDirection(std::string direction)
{
  if (direction == "left")
  {
    return "right";
  } else if (direction == "right")
  {
    return "left";
  } else if (direction == "top")
  {
    return "bottom";
  } else
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

