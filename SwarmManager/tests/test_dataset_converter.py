# tests/test_converter.py

from swarmmanager.dataset_converter import DatasetConverter
from swarmmanager.constants import DATASET_A_IN_FILE
from dataset_a_assertions import basic_assertion_dataset_a

def test_dataset_loading():
    warehouse = DatasetConverter.from_file(DATASET_A_IN_FILE)
    basic_assertion_dataset_a(warehouse)