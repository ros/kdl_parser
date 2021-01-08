from ament_xmllint.main import main
import pytest


@pytest.mark.xmllint
@pytest.mark.linter
def test_xmllint():
    rc = main(argv=[])
    assert rc == 0, "Found errors"
