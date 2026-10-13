import unittest

from stm8_target_check import (
    FLASH_BUDGET,
    FLASH_END,
    FLASH_START,
    RAM_BUDGET,
    RAM_SIZE,
    check_size_limits,
)


class Stm8SizeLimitTests(unittest.TestCase):
    def test_accepts_values_at_regression_budgets(self):
        check_size_limits(FLASH_BUDGET, RAM_BUDGET)

    def test_rejects_flash_regression(self):
        with self.assertRaisesRegex(SystemExit, "flash regression budget"):
            check_size_limits(FLASH_BUDGET + 1, RAM_BUDGET)

    def test_rejects_ram_regression(self):
        with self.assertRaisesRegex(SystemExit, "RAM regression budget"):
            check_size_limits(FLASH_BUDGET, RAM_BUDGET + 1)

    def test_reports_architectural_limits_first(self):
        with self.assertRaisesRegex(SystemExit, "flash capacity"):
            check_size_limits(FLASH_END - FLASH_START + 1, RAM_BUDGET)
        with self.assertRaisesRegex(SystemExit, "RAM capacity"):
            check_size_limits(FLASH_BUDGET, RAM_SIZE + 1)


if __name__ == "__main__":
    unittest.main()
