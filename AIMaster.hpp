//----------------------------------------------------------------------------------------------------------------------
// AIMaster.hpp
//
// TODO
//
// Authors: Triochter Bande (Grill Matthias, Killer Lorenz, Nagy Lukas)
//----------------------------------------------------------------------------------------------------------------------
//

#ifndef A2_AIMASTER_HPP
#define A2_AIMASTER_HPP

#include <stddef.h>
#include <vector>
#include <set>
#include <map>

class Game;
class Player;
class Tile;
class Treasure;

enum class Direction;

typedef std::pair<int, int> Coordinates;
typedef std::pair<Tile*, Coordinates> TileAndCoordinates;

class AIMaster
{
private:
  Game& game_;

  bool executeAllCommands(std::vector<std::vector<std::string>>& commands);

  //Pathfinding
  bool addNeighbors(Tile* current_tile, size_t current_row, size_t current_column, Tile* to_tile,
                    std::set<Tile*>& discovered, std::map<Tile*, Coordinates>& unvisited);
  Tile* chooseNextTile(size_t to_row, size_t to_column, std::map<Tile*, Coordinates>&
  unvisited, size_t& current_row, size_t& current_column);
  std::vector<TileAndCoordinates> getReachableNeighborsOfTile(Tile* tile, size_t row, size_t column);
  int calculateRowChangeInDirection(Direction direction);
  int calculateColumnChangeInDirection(Direction direction);

  // go
  void playerGo(std::vector<std::vector<std::string>>& commands, Coordinates desired_coordinates);
  Coordinates getDesiredCoordinates();
  Coordinates getHomeBaseCoordinates();
  Coordinates getTreasureCoordinates(Treasure* treasure);


  // insert
  void makeInsert(std::vector<std::vector<std::string>>& commands, Coordinates& desired_coordinates);
  bool testInsert(std::string direction, std::string index, Coordinates& desired_coordinates);
  void fakeInsertRow(std::string direction, std::string index);
  void fakeInsertColumn(std::string direction, std::string index);
  std::string getOppositeDirection(std::string direction);
  void undoFakeInsert(std::string direction, std::string index);
  bool checkLastInsert(std::string direction, std::string index);
  void randomInsert(std::vector<std::vector<std::string>>& commands);

public:
  AIMaster(Game& game);

  ~AIMaster() = default;
  AIMaster(const AIMaster& ai_master) = delete;
  AIMaster& operator=(const AIMaster& ai_master) = delete;

  bool isConnected(Player* current_player, size_t to_row, size_t to_column);
  bool playAI();
};

#endif // A2_AIMASTER_HPP
