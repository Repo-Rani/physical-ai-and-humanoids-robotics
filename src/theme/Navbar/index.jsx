import React from 'react';
import OriginalNavbar from '@theme-original/Navbar';
import NavbarTitle from './Title';

const Navbar = (props) => {
  return (
    <nav className="navbar shadow-md transition-all duration-300">
      <OriginalNavbar {...props} />
    </nav>
  );
};

export default Navbar;