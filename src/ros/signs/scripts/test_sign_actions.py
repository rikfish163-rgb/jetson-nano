#!/usr/bin/env python3
import unittest

from sign_actions import LABELS, action_for_index, action_for_label


class SignActionTest(unittest.TestCase):
    def test_seven_class_label_order(self):
        self.assertEqual(
            LABELS,
            ["red", "green", "straight", "left", "right", "uturn", "park"],
        )

    def test_actions_by_label(self):
        expected = {
            "red": (0, 0),
            "green": (20, 0),
            "straight": (20, 0),
            "left": (20, 10),
            "right": (20, -10),
            "uturn": (20, 11),
            "park": (0, 0),
        }
        for label, action in expected.items():
            self.assertEqual(action_for_label(label), action)

    def test_actions_by_index(self):
        self.assertEqual(action_for_index(0), ("red", 0, 0))
        self.assertEqual(action_for_index(1), ("green", 20, 0))
        self.assertEqual(action_for_index(2), ("straight", 20, 0))
        self.assertEqual(action_for_index(3), ("left", 20, 10))
        self.assertEqual(action_for_index(4), ("right", 20, -10))
        self.assertEqual(action_for_index(5), ("uturn", 20, 11))
        self.assertEqual(action_for_index(6), ("park", 0, 0))


if __name__ == "__main__":
    unittest.main()
