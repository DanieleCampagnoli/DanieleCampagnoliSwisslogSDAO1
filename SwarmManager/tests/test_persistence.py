# tests/test_converter.py

from swarmmanager.dataset_converter import DatasetConverter
from swarmmanager.constants import DATASET_A_IN_FILE
from swarmmanager.bo import init_db
from swarmmanager.bo import clean_db
from swarmmanager.bo import SwarmManagerWarehouse
from dataset_a_assertions import basic_assertion_dataset_a

def test_save_warehouse_entities():
    clean_db("db/swarm_test.db")
    warehouse = DatasetConverter.from_file(DATASET_A_IN_FILE)
    basic_assertion_dataset_a(warehouse)
    Session = init_db("sqlite:///db/swarm_test.db")
    session = Session()
    # Save everything
    session.add(warehouse)
    session.commit()
    # query the warehouse object
    warehouseFromDb = session.query(SwarmManagerWarehouse)\
    .filter(SwarmManagerWarehouse.id == 1)\
    .first()
    basic_assertion_dataset_a(warehouseFromDb)
    