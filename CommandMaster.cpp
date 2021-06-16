//----------------------------------------------------------------------------------------------------------------------
// CommandMaster.cpp
//
// The CommandMaster handles all commands entered in the command line
// and calls the corresponding functions in other classes
//
// Authors: Triochter Bande (Grill Matthias, Killer Lorenz, Nagy Lukas)
//----------------------------------------------------------------------------------------------------------------------
//

#include "CommandMaster.hpp"
#include "Game.hpp"
#include "Item.hpp"
#include "ItemTile.hpp"
#include "Player.hpp"
#include "PrintMaster.hpp"
#include "StartTile.hpp"
#include "Tile.hpp"
#include "Treasure.hpp"
#include "TreasureTile.hpp"
#include "AIMaster.hpp"

#include <algorithm>
#include <iostream>
#include <sstream>

CommandMaster::CommandMaster(Game& game) : game_{game} {};

bool CommandMaster::handleCommand()
{
  std::string command;
  std::getline(std::cin, command);
  std::vector<std::string> tokens = tokenize(command);
  return executeCommand(tokens);
}

std::vector<std::string> CommandMaster::tokenize(std::string input)
{
  std::vector<std::string> tokens;
  std::istringstream tokens_stream(input);
  std::string token;
  while (tokens_stream >> token)
  {
    tokens.push_back(token);
  }
  return tokens;
}

bool CommandMaster::executeCommand(std::vector<std::string>& tokens)
{
  std::string command = "";
  if (tokens.size() > 0)
  {
    command = tokens[COMMAND_INDEX];
  }

  if (std::cin.eof())
  {
    game_.quitGame();
    return true;
  }
  else if (command == "showtreasure" || command == "st")
  {
    showTreasure(tokens);
  }
  else if (command == "hidetreasure" || command == "ht")
  {
    hideTreasure(tokens);
  }
  else if (command == "showfreetile" || command == "sft")
  {
    showFreeTile(tokens);
  }
  else if (command == "rotate")
  {
    rotateFreeTile(tokens);
  }
  else if (command == "insert" || command == "i")
  {
    insert(tokens);
  }
  else if (command == "gamefield" || command == "g")
  {
    gameField(tokens);
  }
  else if (command == "finish" || command == "f")
  {
    return finish(tokens);
  }
  else if (command == "item" && game_.getBonusItemsFlag())
  {
    useItem(tokens);
  }
  else if (std::find(PLAYER_MOVEMENT.begin(), PLAYER_MOVEMENT.end(), command) != PLAYER_MOVEMENT.end())
  {
    movePlayer(tokens);
  }
  else if(command == "play")
  {
    play(tokens);
  }
  else if(command == "whoami")
  {
    whoAmI(tokens);
  }
  else if (command == "quit" || command == "exit")
  {
    if (tokens.size() == 1)
    {
      game_.quitGame();
      return true;
    }
    else
    {
      game_.getPrintMaster()->commandTakesNoArguments();
    }
  }
  else if (command == "")
  {
    return false;
  }
  else
  {
    game_.getPrintMaster()->invalidCommand(command);
  }
  return false;
}

void CommandMaster::reset()
{
  inserted_ = false;
  show_treasure_ = false;
}

void CommandMaster::showTreasure(std::vector<std::string> tokens)
{
  if (tokens.size() > 1)
  {
    game_.getPrintMaster()->commandTakesNoArguments();
  }
  else
  {
    show_treasure_ = true;
    if (show_gamefield_)
    {
      game_.getPrintMaster()->printGame();
    }
    else
    {
      game_.getPrintMaster()->printTreasure();
    }
  }
}

void CommandMaster::hideTreasure(std::vector<std::string> tokens)
{
  if (tokens.size() == 1)
  {
    show_treasure_ = false;
    game_.getPrintMaster()->resetUI();
  }
  else
  {
    game_.getPrintMaster()->commandTakesNoArguments();
  }
}

void CommandMaster::showFreeTile(std::vector<std::string> tokens)
{
  if (tokens.size() == 1)
  {
    game_.getPrintMaster()->printFreeTile();
  }
  else
  {
    game_.getPrintMaster()->commandTakesNoArguments();
  }
}

void CommandMaster::rotateFreeTile(std::vector<std::string> tokens)
{
  if (!inserted_)
  {
    if (tokens.size() == 2)
    {
      std::string direction = tokens[FIRST_ARGUMENT_INDEX];
      if (direction == "l" || direction == "left")
      {
        game_.getFreeTile()->rotate(Direction::LEFT);
      }
      else if (direction == "r" || direction == "right")
      {
        game_.getFreeTile()->rotate(Direction::RIGHT);
      }
      else
      {
        game_.getPrintMaster()->invalidParameter(tokens[FIRST_ARGUMENT_INDEX]);
      }
    }
    else
    {
      game_.getPrintMaster()->wrongNumberArguments();
    }
  }
  else
  {
    game_.getPrintMaster()->commandNotAllowed(tokens[COMMAND_INDEX]);
  }
}

void CommandMaster::gameField(std::vector<std::string> tokens)
{
  if (tokens.size() > 2)
  {
    game_.getPrintMaster()->wrongNumberArguments();
  }
  else if (tokens.size() == 2)
  {
    if (tokens[FIRST_ARGUMENT_INDEX] == "on")
    {
      show_gamefield_ = true;
    }
    else if (tokens[FIRST_ARGUMENT_INDEX] == "off")
    {
      show_gamefield_ = false;
    }
    else
    {
      game_.getPrintMaster()->invalidParameter(tokens[FIRST_ARGUMENT_INDEX]);
    }
  }
  else
  {
    game_.getPrintMaster()->printGame();
  }
}

bool CommandMaster::finish(std::vector<std::string> tokens)
{
  if (inserted_)
  {
    if (tokens.size() != 1)
    {
      game_.getPrintMaster()->commandTakesNoArguments();
      return false;
    }

    Treasure* treasure_to_collect;
    if (checkCollectTreasure(treasure_to_collect))
    {
      currentPlayerCollectTreasure(treasure_to_collect);
    }

    Item* item_to_collect;
    if (checkCollectItem(item_to_collect))
    {
      currentPlayerCollectItem(item_to_collect);
    }

    if (checkWin())
    {
      return true;
    }

    game_.getCurrentPlayer()->setTied(false);
    game_.nextPlayer();
    return true;
  }
  else
  {
    game_.getPrintMaster()->commandNotAllowed(tokens[COMMAND_INDEX]);
    return false;
  }
}

bool CommandMaster::checkCollectTreasure(Treasure*& treasure_to_collect)
{
  size_t player_row = game_.getCurrentPlayer()->getRow();
  size_t player_column = game_.getCurrentPlayer()->getCol();

  if (game_.getBoard()[player_row][player_column]->hasTreasure())
  {
    TreasureTile* tile = dynamic_cast<TreasureTile*>(game_.getBoard()[player_row][player_column]);
    if (currentPlayerNeedsTreasure(tile->getTreasure()))
    {
      treasure_to_collect = tile->getTreasure();
      return true;
    }
  }
  return false;
}

bool CommandMaster::checkCollectItem(Item*& item_to_collect)
{
  size_t player_row = game_.getCurrentPlayer()->getRow();
  size_t player_column = game_.getCurrentPlayer()->getCol();

  if (game_.getBoard()[player_row][player_column]->hasItem())
  {
    ItemTile* tile = dynamic_cast<ItemTile*>(game_.getBoard()[player_row][player_column]);
    if (game_.getCurrentPlayer()->getItem() == nullptr)
    {
      item_to_collect = tile->getItem();
      return true;
    }
  }
  return false;
}

bool CommandMaster::checkWin()
{
  size_t player_row = game_.getCurrentPlayer()->getRow();
  size_t player_column = game_.getCurrentPlayer()->getCol();

  if (game_.isCorner(player_row, player_column))
  {
    StartTile* tile = dynamic_cast<StartTile*>(game_.getBoard()[player_row][player_column]);
    if (game_.getCurrentPlayer()->getPlayerColor() == tile->getPlayerColor())
    {
      if (game_.getCurrentPlayer()->getCoveredStackRef().empty())
      {
        std::string color = game_.getCurrentPlayer()->getPlayerColorAsString();
        std::transform(color.begin(), color.end(), color.begin(), ::toupper);

        game_.getPrintMaster()->printWin(color);

        game_.quitGame();
        return true;
      }
    }
  }

  return false;
}

bool CommandMaster::currentPlayerNeedsTreasure(Treasure* treasure)
{
  std::vector<Treasure*>& stack = game_.getCurrentPlayer()->getCoveredStackRef();
  if (stack.empty() || stack.back() != treasure)
  {
    return false;
  }
  return true;
}

void CommandMaster::currentPlayerCollectTreasure(Treasure* treasure)
{
  treasure->setFound(true);
  std::vector<Treasure*>& stack = game_.getCurrentPlayer()->getCoveredStackRef();
  stack.pop_back();
  game_.getCurrentPlayer()->incrementNrFoundTreasures();
}

void CommandMaster::currentPlayerCollectItem(Item* item)
{
  item->setFound(true);
  game_.getCurrentPlayer()->setItem(item);
}

void CommandMaster::insert(std::vector<std::string> tokens)
{
  if (!inserted_)
  {
    if (tokens.size() == 3)
    {
      if (checkInsertParameter(tokens))
      {
        insertTile(tokens);
        inserted_ = true;
      }
    }
    else
    {
      game_.getPrintMaster()->wrongNumberArguments();
    }
  }
  else
  {
    game_.getPrintMaster()->commandNotAllowed(tokens[COMMAND_INDEX]);
  }
}

bool CommandMaster::checkInsertParameter(std::vector<std::string> tokens)
{
  if (isValidInsertDirection(tokens[FIRST_ARGUMENT_INDEX]))
  {
    size_t row_col;
    try
    {
      stringToSizeT(tokens[SECOND_ARGUMENT_INDEX], row_col);
    }
    catch (...)
    {
      game_.getPrintMaster()->invalidParameter(tokens[SECOND_ARGUMENT_INDEX]);
      return false;
    }
    if (row_col > 0 && row_col < BOARD_SIZE + 1)
    {
      if (isInMoveableRowOrCol(row_col))
      {
        if (checkLastInsert(tokens))
        {
          return true;
        }
        else
        {
          game_.getPrintMaster()->commandNotAllowed(tokens);
        }
      }
      else
      {
        game_.getPrintMaster()->invalidInsertPosition();
      }
    }
    else
    {
      game_.getPrintMaster()->invalidParameter(tokens[SECOND_ARGUMENT_INDEX]);
    }
  }
  else
  {
    game_.getPrintMaster()->invalidParameter(tokens[FIRST_ARGUMENT_INDEX]);
  }
  return false;
}

bool CommandMaster::isValidInsertDirection(std::string direction)
{
  if (direction == "t" || direction == "top")
  {
    return true;
  }
  else if (direction == "l" || direction == "left")
  {
    return true;
  }
  else if (direction == "b" || direction == "bottom")
  {
    return true;
  }
  else if (direction == "r" || direction == "right")
  {
    return true;
  }
  return false;
}

bool CommandMaster::isInMoveableRowOrCol(size_t row_col)
{
  return ((row_col - 1) % 2 == 1);
}

bool CommandMaster::checkLastInsert(std::vector<std::string> tokens)
{
  if (tokens[FIRST_ARGUMENT_INDEX] == "t" || tokens[FIRST_ARGUMENT_INDEX] == "top")
  {
    return compareLastInsert("b", "bottom", tokens[SECOND_ARGUMENT_INDEX]);
  }
  else if (tokens[FIRST_ARGUMENT_INDEX] == "l" || tokens[FIRST_ARGUMENT_INDEX] == "left")
  {
    return compareLastInsert("r", "right", tokens[SECOND_ARGUMENT_INDEX]);
  }
  else if (tokens[FIRST_ARGUMENT_INDEX] == "b" || tokens[FIRST_ARGUMENT_INDEX] == "bottom")
  {
    return compareLastInsert("t", "top", tokens[SECOND_ARGUMENT_INDEX]);
  }
  else
  {
    return compareLastInsert("l", "left", tokens[SECOND_ARGUMENT_INDEX]);
  }
}

bool CommandMaster::compareLastInsert(std::string direction, std::string alias, std::string row_col)
{
  if (last_insert_direction_ == direction || last_insert_direction_ == alias)
  {
    if (last_insert_row_col_ == static_cast<size_t>(std::stoi(row_col)))
    {
      return false;
    }
  }
  return true;
}

void CommandMaster::insertTile(std::vector<std::string> tokens)
{
  if (tokens[FIRST_ARGUMENT_INDEX] == "t" || tokens[FIRST_ARGUMENT_INDEX] == "top")
  {
    insertColumn(tokens);
  }
  else if (tokens[FIRST_ARGUMENT_INDEX] == "l" || tokens[FIRST_ARGUMENT_INDEX] == "left")
  {
    insertRow(tokens);
  }
  else if (tokens[FIRST_ARGUMENT_INDEX] == "b" || tokens[FIRST_ARGUMENT_INDEX] == "bottom")
  {
    insertColumn(tokens);
  }
  else if (tokens[FIRST_ARGUMENT_INDEX] == "r" || tokens[FIRST_ARGUMENT_INDEX] == "right")
  {
    insertRow(tokens);
  }
  last_insert_direction_ = tokens[FIRST_ARGUMENT_INDEX];
  last_insert_row_col_ = std::stoi(tokens[SECOND_ARGUMENT_INDEX]);
  game_.getPrintMaster()->printGame();
}

void CommandMaster::insertRow(std::vector<std::string> tokens)
{
  size_t row = std::stoi(tokens[SECOND_ARGUMENT_INDEX]) - 1;
  size_t last_tile_index = BOARD_SIZE - 1;
  Tile* temp_free_tile = game_.getFreeTile();
  std::vector<std::vector<Tile*>>& board = game_.getBoard();
  if (tokens[FIRST_ARGUMENT_INDEX] == "l" || tokens[FIRST_ARGUMENT_INDEX] == "left")
  {
    game_.setFreeTile(board[row][last_tile_index]);
    for (size_t column = last_tile_index; column > 0; column--)
    {
      board[row][column] = board[row][column - 1];
      playersUpdateRowColumn(board[row][column]->getPlayers(), row, column);
    }
    board[row][0] = temp_free_tile;
    movePlayersToTile(game_.getFreeTile(), row, 0);
  }
  else
  {
    game_.setFreeTile(board[row][0]);
    for (size_t column = 0; column < last_tile_index; column++)
    {
      board[row][column] = board[row][column + 1];
      playersUpdateRowColumn(board[row][column]->getPlayers(), row, column);
    }
    board[row][BOARD_SIZE - 1] = temp_free_tile;
    movePlayersToTile(game_.getFreeTile(), row, BOARD_SIZE - 1);
  }
}

void CommandMaster::insertColumn(std::vector<std::string> tokens)
{
  size_t column = std::stoi(tokens[SECOND_ARGUMENT_INDEX]) - 1;
  size_t last_tile_index = BOARD_SIZE - 1;
  Tile* temp_free_tile = game_.getFreeTile();
  std::vector<std::vector<Tile*>>& board = game_.getBoard();

  if (tokens[FIRST_ARGUMENT_INDEX] == "t" || tokens[FIRST_ARGUMENT_INDEX] == "top")
  {
    game_.setFreeTile(board[last_tile_index][column]);
    for (size_t row = BOARD_SIZE - 1; row > 0; row--)
    {
      board[row][column] = board[row - 1][column];
      playersUpdateRowColumn(board[row][column]->getPlayers(), row, column);
    }
    board[0][column] = temp_free_tile;
    movePlayersToTile(game_.getFreeTile(), 0, column);
  }
  else
  {
    game_.setFreeTile(board[0][column]);
    for (size_t row = 0; row < BOARD_SIZE - 1; row++)
    {
      board[row][column] = board[row + 1][column];
      playersUpdateRowColumn(board[row][column]->getPlayers(), row, column);
    }
    board[BOARD_SIZE - 1][column] = temp_free_tile;
    movePlayersToTile(game_.getFreeTile(), BOARD_SIZE - 1, column);
  }
}

void CommandMaster::movePlayersToTile(Tile* from, size_t row, size_t column)
{
  for (Player* player : from->getPlayers())
  {
    game_.getBoard()[row][column]->addPlayer(player);
    player->setRowCol(row, column);
    from->removePlayer(player->getPlayerColorAsString());
  }
}

void CommandMaster::playersUpdateRowColumn(std::vector<Player*> players, size_t row, size_t column)
{
  for (Player* player : players)
  {
    player->setRow(row);
    player->setCol(column);
  }
}

void CommandMaster::movePlayer(std::vector<std::string> tokens)
{
  if (inserted_)
  {
    if (checkMoveInput(tokens))
    {
      Direction direction = getDirection(tokens);
      if (direction != Direction::UNDEFINED)
      {
        int movement = getAmount(tokens);
        if (movement > 0)
        {
          if (!game_.getCurrentPlayer()->getTied())
          {
            int row_modifier = 0;
            int col_modifier = 0;
            getMovementModifier(direction, row_modifier, col_modifier);
            if (isMovePossible(direction, movement, row_modifier, col_modifier))
            {
              moveInDirection(game_.getCurrentPlayer(), movement, row_modifier, col_modifier);
              game_.getPrintMaster()->printGameIfNecessary();
            }
            else
            {
              game_.getPrintMaster()->impossibleMove();
            }
          }
          else
          {
            game_.getPrintMaster()->playerRoped();
          }
        }
        else if (movement == 0)
        {
          game_.getPrintMaster()->invalidParameter(tokens[SECOND_ARGUMENT_INDEX]);
        }
      }
      else
      {
        checkPathFinding(tokens);
      }
    }
  }
  else
  {
    moveNotAllowed(tokens[COMMAND_INDEX]);
  }
}

void CommandMaster::checkPathFinding(std::vector<std::string> tokens)
{
  if(tokens.size() == 3)
  {
    size_t row;
    size_t column;

    try
    {
      stringToSizeT(tokens[FIRST_ARGUMENT_INDEX], row);
    }
    catch (...)
    {
      game_.getPrintMaster()->invalidParameter(tokens[FIRST_ARGUMENT_INDEX]);
      return;
    }

    try
    {
      stringToSizeT(tokens[SECOND_ARGUMENT_INDEX], column);
    }
    catch (...)
    {
      game_.getPrintMaster()->invalidParameter(tokens[SECOND_ARGUMENT_INDEX]);
      return;
    }

    if (row > 0 && row <= BOARD_SIZE)
    {
      if (column > 0 && column <= BOARD_SIZE)
      {
        if (game_.getAIMaster()->isConnected(game_.getCurrentPlayer(), row, column))
        {
          teleportPlayer(game_.getCurrentPlayer(), row, column);
        }
        else
        {
          game_.getPrintMaster()->impossibleMove();
        }
      }
      else
      {
        game_.getPrintMaster()->invalidParameter(tokens[SECOND_ARGUMENT_INDEX]);
      }
    }
    else
    {
      game_.getPrintMaster()->invalidParameter(tokens[FIRST_ARGUMENT_INDEX]);
    }
  }
  else if (tokens.size() == 2)
  {
    game_.getPrintMaster()->invalidParameter(tokens[FIRST_ARGUMENT_INDEX]);
  }
  else
  {
    game_.getPrintMaster()->wrongNumberArguments();
  }
}

void CommandMaster::teleportPlayer(Player* current_player, size_t to_row, size_t to_column)
{
  to_row--;
  to_column--;

  Tile* current_tile = game_.getBoard()[current_player->getRow()][current_player->getCol()];
  Tile* new_tile = game_.getBoard()[to_row][to_column];

  current_tile->removePlayer(current_player->getPlayerColorAsString());
  new_tile->addPlayer(current_player);

  current_player->setRowCol(to_row, to_column);
  game_.getPrintMaster()->printGameIfNecessary();
}

bool CommandMaster::checkMoveInput(std::vector<std::string> tokens)
{
  if (tokens[COMMAND_INDEX] == "go")
  {
    if (tokens.size() > 1 && tokens.size() < 4)
    {
      return true;
    }
    else
    {
      game_.getPrintMaster()->wrongNumberArguments();
      return false;
    }
  }
  else if (tokens.size() == 1)
  {
    return true;
  }
  else
  {
    game_.getPrintMaster()->commandTakesNoArguments();
    return false;
  }
}

Direction CommandMaster::getDirection(std::vector<std::string> tokens)
{
  if (tokens[COMMAND_INDEX] == "go")
  {
    if (tokens[MOVEMENT_DIRECTION_INDEX] == "up")
    {
      return Direction::TOP;
    }
    else if (tokens[MOVEMENT_DIRECTION_INDEX] == "left")
    {
      return Direction::LEFT;
    }
    else if (tokens[MOVEMENT_DIRECTION_INDEX] == "down")
    {
      return Direction::BOTTOM;
    }
    else if (tokens[MOVEMENT_DIRECTION_INDEX] == "right")
    {
      return Direction::RIGHT;
    }
  }
  else
  {
    for (size_t index = 0; index < PLAYER_MOVEMENT.size(); index++)
    {
      if (tokens[COMMAND_INDEX] == PLAYER_MOVEMENT[index])
      {
        if (index < MOVE_TOP)
        {
          return Direction::TOP;
        }
        else if (index < MOVE_LEFT)
        {
          return Direction::LEFT;
        }
        else if (index < MOVE_BOTTOM)
        {
          return Direction::BOTTOM;
        }
        else if (index < MOVE_RIGHT)
        {
          return Direction::RIGHT;
        }
      }
    }
  }
  return Direction::UNDEFINED;
}

int CommandMaster::getAmount(std::vector<std::string> tokens)
{
  size_t amount = 0;
  if (tokens[COMMAND_INDEX] == "go" && tokens.size() == 3)
  {
    try
    {
      stringToSizeT(tokens[SECOND_ARGUMENT_INDEX], amount);
    }
    catch (std::out_of_range)
    {
      if (is_digits(tokens[SECOND_ARGUMENT_INDEX]))
      {
        game_.getPrintMaster()->impossibleMove();
      }
      else
      {
        game_.getPrintMaster()->invalidParameter(tokens[SECOND_ARGUMENT_INDEX]);
      }
      return -1;
    }
    catch (std::invalid_argument)
    {
      game_.getPrintMaster()->invalidParameter(tokens[SECOND_ARGUMENT_INDEX]);
      return -1;
    }
    return amount;
  }
  else
  {
    return 1;
  }
}

void CommandMaster::getMovementModifier(Direction direction, int& row_modifier, int& col_modifier)
{
  row_modifier = (static_cast<size_t>(direction) + 1) % 2;
  col_modifier = static_cast<size_t>(direction) % 2;
  if (direction == Direction::LEFT)
  {
    col_modifier = col_modifier * -1;
  }
  if (direction == Direction::TOP)
  {
    row_modifier = row_modifier * -1;
  }
}

bool CommandMaster::isMovePossible(Direction direction, size_t movement, int& row_modifier, int& col_modifier)
{
  int board_size_int = static_cast<int>(BOARD_SIZE);

  Direction opposite_direction = getOppositeDirection(direction);
  Player* current_player = game_.getCurrentPlayer();
  if (current_player->getUsingLadder())
  {
    int row_location = (movement * row_modifier) + current_player->getRow();
    int col_location = (movement * col_modifier) + current_player->getCol();
    if (row_location >= 0 && row_location < board_size_int && col_location >= 0 && col_location < board_size_int)
    {
      current_player->setUsingLadder(false);
      return true;
    }
  }
  for (size_t index = 0; index <= movement; index++)
  {
    int row = game_.getCurrentPlayer()->getRow() + (index * row_modifier);
    int col = game_.getCurrentPlayer()->getCol() + (index * col_modifier);

    if (row >= board_size_int || col >= board_size_int || row < 0 || col < 0)
    {
      return false;
    }
    Tile* current_tile = game_.getBoard()[row][col];
    bool wall_in_front = current_tile->isWallInDirection(direction);
    bool wall_behind = current_tile->isWallInDirection(opposite_direction);
    if (index == 0 && wall_in_front)
    {
      return false;
    }
    else if (index != 0 && index != movement && (wall_behind || wall_in_front))
    {
      return false;
    }
    else if (index == movement && wall_behind)
    {
      return false;
    }
  }
  return true;
}

void CommandMaster::moveInDirection(Player* player, size_t movement, int& row_modifier, int& col_modifier)
{
  std::string player_color = player->getPlayerColorAsString();
  size_t old_row = player->getRow();
  size_t old_col = player->getCol();
  int new_row = player->getRow() + (movement * row_modifier);
  int new_col = player->getCol() + (movement * col_modifier);
  player->setRow(new_row);
  player->setCol(new_col);
  game_.getBoard()[old_row][old_col]->removePlayer(player_color);
  game_.getBoard()[new_row][new_col]->addPlayer(player);
}

Direction CommandMaster::getOppositeDirection(Direction direction)
{
  switch (direction)
  {
    case Direction::TOP:
      return Direction::BOTTOM;
    case Direction::LEFT:
      return Direction::RIGHT;
    case Direction::BOTTOM:
      return Direction::TOP;
    case Direction::RIGHT:
      return Direction::LEFT;
    default:
      return Direction::UNDEFINED;
  }
}

void CommandMaster::moveNotAllowed(std::string command)
{
  if (command == PLAYER_MOVEMENT[ARROW_UP_INDEX])
  {
    command = "arrow up";
  }
  else if (command == PLAYER_MOVEMENT[ARROW_LEFT_INDEX])
  {
    command = "arrow left";
  }
  else if (command == PLAYER_MOVEMENT[ARROW_DOWN_INDEX])
  {
    command = "arrow down";
  }
  else if (command == PLAYER_MOVEMENT[ARROW_RIGHT_INDEX])
  {
    command = "arrow right";
  }
  game_.getPrintMaster()->commandNotAllowed(command);
}

bool CommandMaster::getShowTreasure()
{
  return show_treasure_;
}

bool CommandMaster::stringToSizeT(std::string token, size_t& number)
{
  if (!is_digits(token))
  {
    throw std::invalid_argument("");
  }
  else
  {
    number = std::stoi(token);
  }
  return true;
}

bool CommandMaster::getShowGamefield()
{
  return show_gamefield_;
}

void CommandMaster::useItem(std::vector<std::string> tokens)
{
  if (tokens.size() == 1)
  {
    Player* current_player = game_.getCurrentPlayer();
    if (current_player->getItem() != nullptr)
    {
      current_player->getItem()->use();
    }
    else
    {
      game_.getPrintMaster()->noItem();
    }
  }
  else
  {
    game_.getPrintMaster()->commandTakesNoArguments();
  }
}

bool CommandMaster::is_digits(const std::string& str)
{
  return std::all_of(str.begin(), str.end(), ::isdigit);
}

void CommandMaster::play(std::vector<std::string> tokens)
{
  if(tokens.size() == 1)
  {
    game_.getAIMaster()->playAI();
  }
  else
  {
    game_.getPrintMaster()->commandTakesNoArguments();
  }
}

void CommandMaster::whoAmI(std::vector<std::string> tokens)
{
  if(tokens.size() == 1)
  {
    game_.getPrintMaster()->printAIName();
  }
  else
  {
    game_.getPrintMaster()->commandTakesNoArguments();
  }
}
