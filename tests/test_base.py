import unittest


class TestOT2_Base(unittest.TestCase):
    pass


class TestImports(TestOT2_Base):
    def test_ot2_driver_import(self):
        import ot2_driver

        assert ot2_driver.__version__


if __name__ == "__main__":
    unittest.main()
