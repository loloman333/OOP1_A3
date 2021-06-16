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

enum class Direction;

typedef std::pair<size_t, size_t> Coordinates;
typedef std::pair<Tile*, std::pair<size_t, size_t>> TileAndCoordinates;

class AIMaster
{
private:
  Game& game_;

public:
  AIMaster(Game& game);

  ~AIMaster() = default;
  AIMaster(const AIMaster& ai_master) = delete;
  AIMaster& operator=(const AIMaster& ai_master) = delete;

  bool isConnected(Player* current_player, size_t to_row, size_t to_column);
  bool addNeighbors(Tile* current_tile, size_t current_row, size_t current_column, Tile* to_tile, 
    std::set<Tile*>& discovered, std::map<Tile*, Coordinates>& unvisited);
  Tile* chooseNextTile(size_t to_row, size_t to_column, std::map<Tile*, Coordinates>& 
    unvisited, size_t& current_row, size_t& current_column);
  std::vector<TileAndCoordinates> getReachableNeighborsOfTile(Tile* tile, size_t row, size_t column);
  int calculateRowChangeInDirection(Direction direction);
  int calculateColumnChangeInDirection(Direction direction);
  void playAI();
};

#endif // A2_AIMASTER_HPP
