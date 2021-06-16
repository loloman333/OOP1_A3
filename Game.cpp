//----------------------------------------------------------------------------------------------------------------------
// Game.hpp
//
// The Game class is the main class of this project.
// It implements the logic and controlls the flow of the game.
//
// Authors: Triochter Bande (Grill Matthias, Killer Lorenz, Nagy Lukas)
//----------------------------------------------------------------------------------------------------------------------
//

#include "Game.hpp"
#include "CommandMaster.hpp"
#include "Item.hpp"
#include "ItemTile.hpp"
#include "NormalTile.hpp"
#include "Player.hpp"
#include "PrintMaster.hpp"
#include "Random.hpp"
#include "StartTile.hpp"
#include "Tile.hpp"
#include "Treasure.hpp"
#include "TreasureTile.hpp"
#include "AIMaster.hpp"

#include <algorithm>

Game::Game()
{
  command_master_ = new CommandMaster(*this);
  print_master_ = new PrintMaster(*this);
  ai_master_ = new AIMaster(*this);
}

Game::~Game()
{
  deleteFreeTile();
  deleteBoard();
  deleteTreasures();
  deleteItemsFromPlayers();
  deletePlayers();
  delete command_master_;
  delete print_master_;
  delete ai_master_;
}

Game& Game::instance()
{
  static Game instance_;
  return instance_;
}

int Game::run()
{
  try
  {
    fillTreasures();
    gameStart();

    fillBoard();
    distributeTreasures();

    while (gameIsRunning())
    {
      print_master_->printGame();
      playRound();
    }
  }
  catch (std::bad_alloc)
  {
    return EXIT_MEMORY_ERROR;
  }
  return EXIT_DEFAULT;
}

void Game::gameStart()
{
  std::cout << UI_WELCOME << std::endl;

  std::string input;
  std::vector<std::string> input_vector;
  size_t player_count = 0;
  while (player_count == 0)
  {

    std::cout << UI_PLAYER_COUNT;
    std::getline(std::cin, input);

    input_vector = command_master_->tokenize(input);
    if (input_vector.size() == 1)
    {
      if (input_vector[PLAYER_COUNT_INDEX] == TWO_PLAYERS)
      {
        player_count = 2;
      }
      else if (input_vector[PLAYER_COUNT_INDEX] == THREE_PLAYERS)
      {
        player_count = 3;
      }
      else if (input_vector[PLAYER_COUNT_INDEX] == FOUR_PLAYERS)
      {
        player_count = 4;
      }
      else
      {
        std::cout << UI_WRONG_COUNT << std::endl;
      }
    }
    else if (std::cin.eof())
    {
      quitGame();
      player_count = 2;
    }
    else
    {
      std::cout << UI_WRONG_COUNT << std::endl;
    }
  }

  for (size_t index = 0; index < player_count; index++)
  {
    addPlayer(Player::player_colors_[index]);
  }
}

void Game::playRound()
{
  Player* player = getCurrentPlayer();
  std::string current_output = player->getPlayerColorAsString();
  std::transform(current_output.begin(), current_output.end(), current_output.begin(), ::toupper);
  current_output += PROMPT;

  bool stop = false;
  while (!stop)
  {
    std::cout << current_output;
    stop = command_master_->handleCommand();
  }
  command_master_->reset();
}

void Game::fillBoard()
{
  size_t treasure_index = 0;
  fillStaticTiles(treasure_index);
  fillVariableTiles(treasure_index);
}

void Game::fillStaticTiles(size_t& treasure_index)
{
  size_t player_index = 0;

  for (size_t row_index = 0; row_index < BOARD_SIZE; row_index++)
  {
    board_.push_back(std::vector<Tile*>{});

    for (size_t col_index = 0; col_index < BOARD_SIZE; col_index++)
    {
      if (isInMoveableLine(row_index, col_index))
      {
        board_[row_index].push_back(nullptr);
      }
      else
      {
        if (isCorner(row_index, col_index))
        {
          Tile* tile = new StartTile{Player::player_colors_[player_index]};

          board_[row_index].push_back(tile);

          if (player_index < players_.size())
          {
            tile->addPlayer(players_[player_index]);
            players_[player_index]->setCol(col_index);
            players_[player_index]->setRow(row_index);
          }

          player_index++;
        }
        else
        {
          TreasureTile* treasure_tile = new TreasureTile(treasures_[treasure_index]);

          board_[row_index].push_back(treasure_tile);
          treasure_index++;
        }
      }
    }
  }
}

void Game::fillVariableTiles(size_t& treasure_index)
{
  std::vector<Tile*> tiles;

  if (bonus_items_)
  {
    addNewItemTilesToVector(tiles);
    addNewNormalTilesToVector(tiles, TileType::L, BONUS_GAME_NORMAL_L_TILES);
    addNewNormalTilesToVector(tiles, TileType::I, BONUS_GAME_NORMAL_I_TILES);
    addNewTreasureTilesToVector(tiles, TileType::T, BONUS_GAME_NORMAL_T_TILES, treasure_index);
  }
  else
  {
    addNewNormalTilesToVector(tiles, TileType::L, NORMAL_GAME_NORMAL_L_TILES);
    addNewNormalTilesToVector(tiles, TileType::I, NORMAL_GAME_NORMAL_I_TILES);
    addNewTreasureTilesToVector(tiles, TileType::T, NORMAL_GAME_NORMAL_T_TILES, treasure_index);
  }

  for (size_t row_index = 0; row_index < board_.size(); row_index++)
  {
    for (size_t col_index = 0; col_index < board_[row_index].size(); col_index++)
    {
      if (board_[row_index][col_index] == nullptr)
      {
        size_t random_index = Oop::Random::getInstance().getRandomCard(tiles.size()) - 1;
        board_[row_index][col_index] = tiles[random_index];

        size_t random_rotation = Oop::Random::getInstance().getRandomOrientation();
        board_[row_index][col_index]->setRotation((Rotation) random_rotation);

        tiles.erase(tiles.begin() + random_index);
      }
    }
  }
  free_tile_ = tiles[0];
}

void Game::addNewItemTilesToVector(std::vector<Tile*>& vector)
{
  vector.push_back(new ItemTile(TileType::X, new Item(ItemType::BRICKS)));
  vector.push_back(new ItemTile(TileType::O, new Item(ItemType::DYNAMITE)));
  vector.push_back(new ItemTile(TileType::X, new Item(ItemType::LADDER)));
  vector.push_back(new ItemTile(TileType::X, new Item(ItemType::ROPE)));
}

void Game::addNewNormalTilesToVector(std::vector<Tile*>& vector, TileType type, size_t count)
{
  for (size_t i = 0; i < count; i++)
  {
    NormalTile* normal_tile = new NormalTile(type);
    vector.push_back(normal_tile);
  }
}

void Game::addNewTreasureTilesToVector(std::vector<Tile*>& vector, TileType type, size_t count, size_t& treasure_index)
{
  for (size_t i = 0; i < count; i++)
  {
    Tile* tile = new TreasureTile(type, treasures_[treasure_index]);

    treasure_index++;
    vector.push_back(tile);
  }
}

bool Game::isCorner(size_t row_index, size_t col_index)
{
  return !(row_index % (BOARD_SIZE - 1)) && !(col_index % (BOARD_SIZE - 1));
}

bool Game::isInMoveableLine(size_t row_index, size_t col_index)
{
  return (row_index % 2 == 1 || col_index % 2 == 1);
}

void Game::fillTreasures()
{
  size_t id = 1;
  for (std::string treasure_name : TREASURE_NAMES)
  {
    Treasure* treasure = new Treasure(treasure_name, id);

    treasures_.push_back(treasure);
    id++;
  }
}

void Game::addPlayer(PlayerColor color)
{
  Player* player = new Player{color};

  players_.push_back(player);
}

void Game::nextPlayer()
{
  current_player_index_ = (current_player_index_ + 1) % players_.size();
}

void Game::quitGame()
{
  quit_ = true;
}

bool Game::gameIsRunning()
{
  return !quit_;
}

void Game::distributeTreasures()
{
  std::vector<Treasure*> temp_treasures = treasures_;
  for (size_t cards_dealt = 0; cards_dealt < treasures_.size();)
  {
    for (Player* player : players_)
    {
      size_t next_card_index = Oop::Random::getInstance().getRandomCard(temp_treasures.size());
      player->getCoveredStackRef().push_back(temp_treasures[next_card_index - 1]);
      temp_treasures.erase(temp_treasures.begin() + next_card_index - 1);
      cards_dealt++;
    }
  }
}

void Game::deleteBoard()
{
  for (std::vector<Tile*> row : board_)
  {
    for (Tile* tile : row)
    {
      deleteTile(tile);
    }
  }
}

void Game::deleteItemsFromPlayers()
{
  for (Player* player : players_)
  {
    if (player->getItem() != nullptr)
    {
      delete player->getItem();
    }
  }
}

void Game::deleteTile(Tile* tile)
{
  if (tile->hasItem())
  {
    delete dynamic_cast<ItemTile*>(tile)->getItem();
  }
  delete tile;
}

void Game::deleteTreasures()
{
  for (Treasure* treasure : treasures_)
  {
    delete treasure;
  }
}

void Game::deleteFreeTile()
{
  deleteTile(free_tile_);
}

void Game::deletePlayers()
{
  for (Player* player : players_)
  {
    delete player;
  }
}

Tile* Game::getFreeTile()
{
  return free_tile_;
}

bool Game::getBonusItemsFlag()
{
  return bonus_items_;
}

std::vector<std::vector<Tile*>>& Game::getBoard()
{
  return board_;
}

std::vector<Player*>& Game::getPlayers()
{
  return players_;
}

Player* Game::getCurrentPlayer()
{
  return players_[current_player_index_];
}

std::vector<Treasure*>& Game::getTreasures()
{
  return treasures_;
}

CommandMaster* Game::getCommandMaster()
{
  return command_master_;
}

PrintMaster* Game::getPrintMaster()
{
  return print_master_;
}

AIMaster* Game::getAIMaster()
{
  return ai_master_;
}

void Game::setFreeTile(Tile* tile)
{
  free_tile_ = tile;
}

void Game::setBonus(bool choice)
{
  bonus_items_ = choice;
}
