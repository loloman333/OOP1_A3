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
typedef std::pair<Coordinates , size_t> CoordinatesAndWalls;
typedef std::tuple<Tile*, Coordinates, size_t> TileCoordinatesAndWalls;
typedef std::pair<Tile*, Coordinates> TileAndCoordinates;
typedef std::map<Tile *, CoordinatesAndWalls> TileInfoMap;

class AIMaster
{
private:
  Game& game_;

  bool executeAllCommands(std::vector<std::vector<std::string>>& commands);

  //Pathfinding
  void addNeighbors(Tile* current_tile, TileInfoMap& discovered, TileInfoMap& unvisited);
  Tile* chooseNextTile(size_t to_row, size_t to_column, std::map<Tile*, std::pair<Coordinates, size_t>>&
  unvisited, size_t& current_row, size_t& current_column, size_t& shortest_path);
  std::vector<TileCoordinatesAndWalls> getReachableNeighborsOfTile(Tile* tile, CoordinatesAndWalls info);
  int calculateRowChangeInDirection(Direction direction);
  int calculateColumnChangeInDirection(Direction direction);

  // go
  void playerGo(std::vector<std::vector<std::string>>& commands, Coordinates desired_coordinates);
  Coordinates getDesiredCoordinates();
  Coordinates getHomeBaseCoordinates();
  Coordinates getTreasureCoordinates(Treasure* treasure);
  std::vector<TileAndCoordinates> getNeighborsOfTile(size_t row, size_t column);


  // insert
  void makeInsert(std::vector<std::vector<std::string>>& commands, Coordinates& desired_coordinates);
  bool testInsert(std::string direction, std::string index, Coordinates& desired_coordinates);
  void fakeInsertRow(std::string direction, std::string index);
  void fakeInsertColumn(std::string direction, std::string index);
  std::string getOppositeDirection(std::string direction);
  void undoFakeInsert(std::string direction, std::string index);
  bool checkLastInsert(std::string direction, std::string index);
  void randomInsert(std::vector<std::vector<std::string>>& commands);

  // rotation
  void addRotation(std::vector<std::vector<std::string>>& commands, size_t rotation);
  void undoRotation(size_t left_counter, size_t right_counter);

public:
  AIMaster(Game& game);

  ~AIMaster() = default;
  AIMaster(const AIMaster& ai_master) = delete;
  AIMaster& operator=(const AIMaster& ai_master) = delete;

  size_t getWallsToTile(Player* current_player, size_t to_row, size_t to_column);
  bool playAI();
};

#endif // A2_AIMASTER_HPP
