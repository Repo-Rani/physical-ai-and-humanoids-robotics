import React from 'react';
import OriginalNavbar from '@theme-original/Navbar';
import NavbarTitle from './Title';

const Navbar = (props) => {
  return (
    <nav className="">
      <OriginalNavbar {...props} />
    </nav>
  );
};

export default Navbar;