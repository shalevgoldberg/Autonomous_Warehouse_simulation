from abc import ABC, abstractmethod
from typing import Optional, Tuple


class IAppearanceService(ABC):
    """
    Interface for managing robot visual appearance.

    Provides methods to toggle robot material/rgba when carrying shelves,
    enabling visual feedback without affecting physics simulation.
    """

    @abstractmethod
    def set_carrying_appearance(self, robot_id: str, carrying: bool) -> bool:
        """
        Set robot appearance to indicate carrying state.

        Args:
            robot_id: The robot identifier
            carrying: True to show carrying appearance, False to revert to normal

        Returns:
            bool: True if appearance was successfully changed, False otherwise
        """
        pass

    @abstractmethod
    def is_carrying_appearance(self, robot_id: str) -> bool:
        """
        Check if robot currently has carrying appearance.
        """
        pass

    @abstractmethod
    def get_carrying_color(self) -> Tuple[float, float, float, float]:
        """
        Get the RGBA color used for carrying appearance.
        """
        pass

    @abstractmethod
    def is_enabled(self) -> bool:
        """
        Check if appearance service is enabled.
        """
        pass

