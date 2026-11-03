import unittest

import config
from navigation.maze_map import Dir, TileType, step
from simulation.maze_generator import generate
from simulation.sim_robot import SimWorld


def world_for(seed: int, width=8, height=7, **kwargs) -> SimWorld:
    return SimWorld.from_dict(generate(width, height, seed=seed, **kwargs)[0])


class GeneratorTest(unittest.TestCase):
    def test_same_seed_same_maze(self):
        self.assertEqual(generate(8, 7, seed=5)[0], generate(8, 7, seed=5)[0])
        self.assertNotEqual(generate(8, 7, seed=5)[0], generate(8, 7, seed=6)[0])

    def test_counts(self):
        for seed in range(20):
            data, warnings = generate(8, 7, black=2, blue=3, silver=2, zones=1, seed=seed)
            floors = [t["floor"] for t in data["tiles"]]
            with self.subTest(seed=seed):
                self.assertEqual(warnings, [])
                self.assertEqual(floors.count("black"), 2)
                self.assertEqual(floors.count("blue"), 3)
                self.assertEqual(floors.count("silver"), 2)
                self.assertEqual(floors.count("red"), 1)

    def test_everything_reachable_except_black(self):
        for w, h in config.FIELD_SIZES_2026:
            for seed in range(10):
                world = world_for(seed, w, h, black=3, zones=2)
                not_black = {p for p, k in world.tiles.items() if k is not TileType.BLACK}
                with self.subTest(size=(w, h), seed=seed):
                    self.assertEqual(world.reachable_tiles(), not_black)
                    self.assertIsNot(world.tiles[world.start], TileType.BLACK)

    def test_dangerous_zone_only_through_red_tile(self):
        for seed in range(20):
            world = world_for(seed, zones=1)
            red = next(p for p, k in world.tiles.items() if k is TileType.RED)
            outside = world.reachable_tiles(frozenset({TileType.BLACK, TileType.RED}))
            zone = world.reachable_tiles() - outside
            with self.subTest(seed=seed):
                self.assertIn(red, zone)
                self.assertNotIn(world.start, zone)
                self.assertGreaterEqual(len(zone), 4)
                # Von außen führt genau eine offene Verbindung in die Zone: zur roten Kachel
                doors = [(p, d) for p in outside for d in Dir
                         if not world.wall(p, d) and step(p, d) in zone]
                self.assertEqual(len(doors), 1)
                self.assertEqual(step(*doors[0]), red)

    def test_floating_tiles_occur(self):
        with_floating = sum(
            1 for seed in range(30)
            if world_for(seed, 9, 8).reachable_tiles() - world_for(seed, 9, 8).linear_tiles())
        self.assertGreater(with_floating, 10)

    def test_too_small_field_warns(self):
        data, warnings = generate(2, 2, black=5, zones=1, seed=1)
        self.assertTrue(warnings)
        SimWorld.from_dict(data)   # trotzdem gültig


if __name__ == "__main__":
    unittest.main()
