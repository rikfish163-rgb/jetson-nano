import unittest
from prepare import assign_groups, LABELS


class GroupTests(unittest.TestCase):
    def test_burst_never_crosses_split_even_if_original_split_changes(self):
        rows = []
        for label in LABELS:
            for block in range(5):
                for i in range(10):
                    rows.append(dict(label=label, stamp=1000*block+i, old_split='train' if i<5 else 'test'))
        assign_groups(rows)
        for label in LABELS:
            selected = [r for r in rows if r['label']==label]
            self.assertEqual(set(r['split'] for r in selected), {'train','val','test'})
            for group in set(r['group'] for r in selected):
                self.assertEqual(len(set(r['split'] for r in selected if r['group']==group)),1)

    def test_not_enough_groups_fails(self):
        with self.assertRaises(ValueError):
            assign_groups([dict(label='red', stamp=i) for i in range(20)])


if __name__ == '__main__':
    unittest.main()
