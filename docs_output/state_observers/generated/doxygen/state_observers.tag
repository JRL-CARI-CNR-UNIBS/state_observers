<?xml version='1.0' encoding='UTF-8' standalone='yes' ?>
<tagfile doxygen_version="1.9.1">
  <compound kind="class">
    <name>state_observer::KalmanFilter</name>
    <filename>classstate__observer_1_1KalmanFilter.html</filename>
    <base>state_observer::StateObserver</base>
    <member kind="function">
      <type></type>
      <name>KalmanFilter</name>
      <anchorfile>classstate__observer_1_1KalmanFilter.html</anchorfile>
      <anchor>af19ed90f24aaa03d0ee763cb9f117346</anchor>
      <arglist>(const Eigen::MatrixXd &amp;A, const Eigen::MatrixXd &amp;B, const Eigen::MatrixXd &amp;C, const Eigen::MatrixXd &amp;D, const Eigen::VectorXd &amp;initial_state, const Eigen::MatrixXd &amp;Q, const Eigen::MatrixXd &amp;R)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>KalmanFilter</name>
      <anchorfile>classstate__observer_1_1KalmanFilter.html</anchorfile>
      <anchor>ae6d0948f82050ae5867733e1b6e72c83</anchor>
      <arglist>(const Eigen::MatrixXd &amp;A, const Eigen::MatrixXd &amp;B, const Eigen::MatrixXd &amp;C, const Eigen::VectorXd &amp;initial_state, const Eigen::MatrixXd &amp;Q, const Eigen::MatrixXd &amp;R)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>KalmanFilter</name>
      <anchorfile>classstate__observer_1_1KalmanFilter.html</anchorfile>
      <anchor>a643e5490657b0b20700d4eb7453a19d9</anchor>
      <arglist>(const Eigen::MatrixXd &amp;A, const Eigen::MatrixXd &amp;B, const Eigen::MatrixXd &amp;C, const Eigen::MatrixXd &amp;Q, const Eigen::MatrixXd &amp;R)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>KalmanFilter</name>
      <anchorfile>classstate__observer_1_1KalmanFilter.html</anchorfile>
      <anchor>af083f45079c73cf1fe957d257a358ca1</anchor>
      <arglist>(const Eigen::MatrixXd &amp;A, const Eigen::MatrixXd &amp;B, const Eigen::MatrixXd &amp;C, const Eigen::MatrixXd &amp;D, const Eigen::VectorXd &amp;initial_state, const Eigen::MatrixXd &amp;Q, const Eigen::MatrixXd &amp;R, const Eigen::MatrixXd &amp;P0)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>KalmanFilter</name>
      <anchorfile>classstate__observer_1_1KalmanFilter.html</anchorfile>
      <anchor>aef58ecaf3b277fd289c7f1918d591a5f</anchor>
      <arglist>(const Eigen::MatrixXd &amp;A, const Eigen::MatrixXd &amp;B, const Eigen::MatrixXd &amp;C, const Eigen::VectorXd &amp;initial_state, const Eigen::MatrixXd &amp;Q, const Eigen::MatrixXd &amp;R, const Eigen::MatrixXd &amp;P0)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>KalmanFilter</name>
      <anchorfile>classstate__observer_1_1KalmanFilter.html</anchorfile>
      <anchor>a51aff1fb39d614509507e4dcc5678ada</anchor>
      <arglist>(const Eigen::MatrixXd &amp;A, const Eigen::MatrixXd &amp;B, const Eigen::MatrixXd &amp;C, const Eigen::MatrixXd &amp;Q, const Eigen::MatrixXd &amp;R, const Eigen::MatrixXd &amp;P0)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>KalmanFilter</name>
      <anchorfile>classstate__observer_1_1KalmanFilter.html</anchorfile>
      <anchor>ab3e6a7d05608cfd2de49161eb402e28e</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~KalmanFilter</name>
      <anchorfile>classstate__observer_1_1KalmanFilter.html</anchorfile>
      <anchor>a1aa5b67c1da14d16cf6df82f5386d99b</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>set_parameters</name>
      <anchorfile>classstate__observer_1_1KalmanFilter.html</anchorfile>
      <anchor>a3be4d2de5926eb3e611dc601adc9a595</anchor>
      <arglist>(const StateObserverParam::SharedPtr state_observer_params) override</arglist>
    </member>
    <member kind="function">
      <type>Eigen::MatrixXd</type>
      <name>update</name>
      <anchorfile>classstate__observer_1_1KalmanFilter.html</anchorfile>
      <anchor>a27ada2e6b5ee4efbf3bc6f1781413d80</anchor>
      <arglist>(const Eigen::VectorXd &amp;measurement) override</arglist>
    </member>
    <member kind="function">
      <type>Eigen::MatrixXd</type>
      <name>update</name>
      <anchorfile>classstate__observer_1_1KalmanFilter.html</anchorfile>
      <anchor>a5a69f399dc513a1b55650de63900eb3f</anchor>
      <arglist>(const Eigen::VectorXd &amp;measurement, const Eigen::VectorXd &amp;input) override</arglist>
    </member>
    <member kind="function">
      <type>Eigen::VectorXd</type>
      <name>open_loop_update</name>
      <anchorfile>classstate__observer_1_1KalmanFilter.html</anchorfile>
      <anchor>aef744a50f9a03b27df8edc02b3a648dd</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>update_process_covariance</name>
      <anchorfile>classstate__observer_1_1KalmanFilter.html</anchorfile>
      <anchor>a91064d452d8675c680312f49f3651817</anchor>
      <arglist>(const Eigen::MatrixXd &amp;new_Q)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>update_measurement_covariance</name>
      <anchorfile>classstate__observer_1_1KalmanFilter.html</anchorfile>
      <anchor>a85455fc77488c7b7391adabb26b9485c</anchor>
      <arglist>(const Eigen::MatrixXd &amp;new_R)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>update_qr</name>
      <anchorfile>classstate__observer_1_1KalmanFilter.html</anchorfile>
      <anchor>a40810a6cac1a1ebd1e07c9fea35e1866</anchor>
      <arglist>(const Eigen::MatrixXd &amp;new_Q, const Eigen::MatrixXd &amp;new_R)</arglist>
    </member>
    <member kind="function">
      <type>Eigen::VectorXd</type>
      <name>get_state_variance</name>
      <anchorfile>classstate__observer_1_1KalmanFilter.html</anchorfile>
      <anchor>a6f86415ca604956468ad7e7863ea257d</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>Eigen::MatrixXd</type>
      <name>get_Q</name>
      <anchorfile>classstate__observer_1_1KalmanFilter.html</anchorfile>
      <anchor>a858e311c5639b3e0fe137a969c721542</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>Eigen::MatrixXd</type>
      <name>get_R</name>
      <anchorfile>classstate__observer_1_1KalmanFilter.html</anchorfile>
      <anchor>a87c43b2faaaf326a5be1e1511d0ccdc6</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>Eigen::MatrixXd</type>
      <name>get_P0</name>
      <anchorfile>classstate__observer_1_1KalmanFilter.html</anchorfile>
      <anchor>ac35e2976957f06c034f685c0babcc51d</anchor>
      <arglist>() const</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>state_observer::KalmanFilterParam</name>
    <filename>classstate__observer_1_1KalmanFilterParam.html</filename>
    <base>state_observer::StateObserverParam</base>
    <member kind="function">
      <type></type>
      <name>KalmanFilterParam</name>
      <anchorfile>classstate__observer_1_1KalmanFilterParam.html</anchorfile>
      <anchor>a927d572406367413ef896755008de4d6</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~KalmanFilterParam</name>
      <anchorfile>classstate__observer_1_1KalmanFilterParam.html</anchorfile>
      <anchor>acdcc5e097956f80f1a031840b12295bb</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual void</type>
      <name>initialize</name>
      <anchorfile>classstate__observer_1_1KalmanFilterParam.html</anchorfile>
      <anchor>a3840304b32334760f8a207e3ef5c6e86</anchor>
      <arglist>(const rclcpp_lifecycle::LifecycleNode::SharedPtr &amp;node)</arglist>
    </member>
    <member kind="function">
      <type>Eigen::MatrixXd</type>
      <name>get_R</name>
      <anchorfile>classstate__observer_1_1KalmanFilterParam.html</anchorfile>
      <anchor>a8090ec5b7fefcea6e6efb9804adf62ae</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>Eigen::MatrixXd</type>
      <name>get_Q</name>
      <anchorfile>classstate__observer_1_1KalmanFilterParam.html</anchorfile>
      <anchor>af2ee0a0de8f48cd6b1308b103bae5835</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>Eigen::MatrixXd</type>
      <name>get_P0</name>
      <anchorfile>classstate__observer_1_1KalmanFilterParam.html</anchorfile>
      <anchor>a8995c63684626e549c418e089683271b</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>get_type</name>
      <anchorfile>classstate__observer_1_1KalmanFilterParam.html</anchorfile>
      <anchor>a4d554947dd43bfd8b0a3f0bf87da31f7</anchor>
      <arglist>() const override</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>state_observer::Luenberger</name>
    <filename>classstate__observer_1_1Luenberger.html</filename>
    <base>state_observer::StateObserver</base>
    <member kind="function">
      <type></type>
      <name>Luenberger</name>
      <anchorfile>classstate__observer_1_1Luenberger.html</anchorfile>
      <anchor>a16929984163b7f47e97db1d070f290ce</anchor>
      <arglist>(const Eigen::MatrixXd &amp;A, const Eigen::MatrixXd &amp;B, const Eigen::MatrixXd &amp;C, const Eigen::MatrixXd &amp;D, const Eigen::VectorXd &amp;initial_state, const Eigen::MatrixXd &amp;L)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Luenberger</name>
      <anchorfile>classstate__observer_1_1Luenberger.html</anchorfile>
      <anchor>abae9eae252eb9b3755290387fd3ed66b</anchor>
      <arglist>(const Eigen::MatrixXd &amp;A, const Eigen::MatrixXd &amp;B, const Eigen::MatrixXd &amp;C, const Eigen::VectorXd &amp;initial_state, const Eigen::MatrixXd &amp;L)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Luenberger</name>
      <anchorfile>classstate__observer_1_1Luenberger.html</anchorfile>
      <anchor>a4493b2461f323dafb197ac4fcd1f88c0</anchor>
      <arglist>(const Eigen::MatrixXd &amp;A, const Eigen::MatrixXd &amp;B, const Eigen::MatrixXd &amp;C, const Eigen::MatrixXd &amp;L)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Luenberger</name>
      <anchorfile>classstate__observer_1_1Luenberger.html</anchorfile>
      <anchor>abe60dea3226dd881e2df2f4a1cea4a49</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~Luenberger</name>
      <anchorfile>classstate__observer_1_1Luenberger.html</anchorfile>
      <anchor>abee2fc506b66688f26ace1f4ef756f49</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>set_parameters</name>
      <anchorfile>classstate__observer_1_1Luenberger.html</anchorfile>
      <anchor>a990ad8246d6da844e8facd76bac5d948</anchor>
      <arglist>(const StateObserverParam::SharedPtr state_observer_params) override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>set_observer_gain</name>
      <anchorfile>classstate__observer_1_1Luenberger.html</anchorfile>
      <anchor>ab46f7f08840c4688ef9d1e992fadea73</anchor>
      <arglist>(const Eigen::MatrixXd &amp;L)</arglist>
    </member>
    <member kind="function">
      <type>Eigen::MatrixXd</type>
      <name>update</name>
      <anchorfile>classstate__observer_1_1Luenberger.html</anchorfile>
      <anchor>aeef08b1e109c369246e440f7bf92f3e2</anchor>
      <arglist>(const Eigen::VectorXd &amp;measurement) override</arglist>
    </member>
    <member kind="function">
      <type>Eigen::MatrixXd</type>
      <name>update</name>
      <anchorfile>classstate__observer_1_1Luenberger.html</anchorfile>
      <anchor>ad514f468dbfa4e073232e1438c55176a</anchor>
      <arglist>(const Eigen::VectorXd &amp;measurement, const Eigen::VectorXd &amp;input) override</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>state_observer::LuenbergerParam</name>
    <filename>classstate__observer_1_1LuenbergerParam.html</filename>
    <base>state_observer::StateObserverParam</base>
    <member kind="function">
      <type></type>
      <name>LuenbergerParam</name>
      <anchorfile>classstate__observer_1_1LuenbergerParam.html</anchorfile>
      <anchor>a5c535549b5fd8d874942e9f4bbe21f09</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~LuenbergerParam</name>
      <anchorfile>classstate__observer_1_1LuenbergerParam.html</anchorfile>
      <anchor>ab642dffc1b7a8279945c179b51eef70c</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual void</type>
      <name>initialize</name>
      <anchorfile>classstate__observer_1_1LuenbergerParam.html</anchorfile>
      <anchor>a588e53e56e20e684ac6e3d81fb4fd22a</anchor>
      <arglist>(const rclcpp_lifecycle::LifecycleNode::SharedPtr &amp;node)</arglist>
    </member>
    <member kind="function">
      <type>Eigen::MatrixXd</type>
      <name>get_observer_gain</name>
      <anchorfile>classstate__observer_1_1LuenbergerParam.html</anchorfile>
      <anchor>a07953be75baa7bac3a598db85a244b97</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>get_type</name>
      <anchorfile>classstate__observer_1_1LuenbergerParam.html</anchorfile>
      <anchor>a5bd007160b851aa62273de43b2e46282</anchor>
      <arglist>() const override</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>state_observer::StateObserver</name>
    <filename>classstate__observer_1_1StateObserver.html</filename>
    <member kind="function">
      <type></type>
      <name>StateObserver</name>
      <anchorfile>classstate__observer_1_1StateObserver.html</anchorfile>
      <anchor>af7f4758be22113043bcf627614c72b2c</anchor>
      <arglist>(const Eigen::MatrixXd &amp;A, const Eigen::MatrixXd &amp;B, const Eigen::MatrixXd &amp;C, const Eigen::MatrixXd &amp;D, const Eigen::VectorXd &amp;initial_state)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>StateObserver</name>
      <anchorfile>classstate__observer_1_1StateObserver.html</anchorfile>
      <anchor>af264a05c2165ad46ac0a791fc0d81345</anchor>
      <arglist>(const Eigen::MatrixXd &amp;A, const Eigen::MatrixXd &amp;B, const Eigen::MatrixXd &amp;C, const Eigen::VectorXd &amp;initial_state)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>StateObserver</name>
      <anchorfile>classstate__observer_1_1StateObserver.html</anchorfile>
      <anchor>ab7c5cc9c2fe24acf92f418d698f1e080</anchor>
      <arglist>(const Eigen::MatrixXd &amp;A, const Eigen::MatrixXd &amp;B, const Eigen::MatrixXd &amp;C)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>StateObserver</name>
      <anchorfile>classstate__observer_1_1StateObserver.html</anchorfile>
      <anchor>aaa7ed3fc92876429b6b91ce90e42b9d6</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual void</type>
      <name>set_parameters</name>
      <anchorfile>classstate__observer_1_1StateObserver.html</anchorfile>
      <anchor>a862bfeb79def6189ad6446dd023781a8</anchor>
      <arglist>(const StateObserverParam::SharedPtr state_observer_params)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>initialize</name>
      <anchorfile>classstate__observer_1_1StateObserver.html</anchorfile>
      <anchor>ac3ae192893b2899e47dcf6a572924dc1</anchor>
      <arglist>(const Eigen::VectorXd &amp;initial_state)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual Eigen::VectorXd</type>
      <name>open_loop_update</name>
      <anchorfile>classstate__observer_1_1StateObserver.html</anchorfile>
      <anchor>ac6328e22b68efe1f038d7d275aabe0f1</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual Eigen::MatrixXd</type>
      <name>update</name>
      <anchorfile>classstate__observer_1_1StateObserver.html</anchorfile>
      <anchor>a07062cefef35b9ec7bf7fc812e82e826</anchor>
      <arglist>(const Eigen::VectorXd &amp;measurement)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual Eigen::MatrixXd</type>
      <name>update</name>
      <anchorfile>classstate__observer_1_1StateObserver.html</anchorfile>
      <anchor>ab3bf488d4abc29ad203d5a566018aabd</anchor>
      <arglist>(const Eigen::VectorXd &amp;measurement, const Eigen::VectorXd &amp;input)=0</arglist>
    </member>
    <member kind="function">
      <type>Eigen::VectorXd</type>
      <name>get_state</name>
      <anchorfile>classstate__observer_1_1StateObserver.html</anchorfile>
      <anchor>ac9bce8f25518d27fe05e5a76b024892f</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>Eigen::VectorXd</type>
      <name>get_output</name>
      <anchorfile>classstate__observer_1_1StateObserver.html</anchorfile>
      <anchor>a65034cb07db2a389bdcff3eaf4c6fa6f</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual Eigen::VectorXd</type>
      <name>get_state_variance</name>
      <anchorfile>classstate__observer_1_1StateObserver.html</anchorfile>
      <anchor>aabe418247e711fa2d375f602dc5b6dc2</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>state_observer::StateObserverParam</name>
    <filename>classstate__observer_1_1StateObserverParam.html</filename>
    <member kind="function">
      <type></type>
      <name>StateObserverParam</name>
      <anchorfile>classstate__observer_1_1StateObserverParam.html</anchorfile>
      <anchor>a25109f549d5cd7b51aa4e81dca5c0a11</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~StateObserverParam</name>
      <anchorfile>classstate__observer_1_1StateObserverParam.html</anchorfile>
      <anchor>a2621b7eedb1f51878790b442620103b0</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual void</type>
      <name>initialize</name>
      <anchorfile>classstate__observer_1_1StateObserverParam.html</anchorfile>
      <anchor>a878fdd58575488e88f2661024e971649</anchor>
      <arglist>(const rclcpp_lifecycle::LifecycleNode::SharedPtr &amp;node)</arglist>
    </member>
    <member kind="function">
      <type>Eigen::MatrixXd</type>
      <name>get_state_transition_matrix</name>
      <anchorfile>classstate__observer_1_1StateObserverParam.html</anchorfile>
      <anchor>a00ae379c2dfe6dbdf10d18a0528d3623</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>Eigen::MatrixXd</type>
      <name>get_input_matrix</name>
      <anchorfile>classstate__observer_1_1StateObserverParam.html</anchorfile>
      <anchor>a9954e81143a988fd25886cbfa3605616</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>Eigen::MatrixXd</type>
      <name>get_output_matrix</name>
      <anchorfile>classstate__observer_1_1StateObserverParam.html</anchorfile>
      <anchor>ae6a757e174a59ef72874e28ecedb1259</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>Eigen::MatrixXd</type>
      <name>get_feedforward_matrix</name>
      <anchorfile>classstate__observer_1_1StateObserverParam.html</anchorfile>
      <anchor>a6252757736637f0f3453228bea743f84</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>Eigen::VectorXd</type>
      <name>get_initial_state</name>
      <anchorfile>classstate__observer_1_1StateObserverParam.html</anchorfile>
      <anchor>a9fe90d5e66b8e5777d449fb5069af79c</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>Eigen::MatrixXd</type>
      <name>get_A</name>
      <anchorfile>classstate__observer_1_1StateObserverParam.html</anchorfile>
      <anchor>a7e2d98ef1a6ff08144c82834adc7a095</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>Eigen::MatrixXd</type>
      <name>get_B</name>
      <anchorfile>classstate__observer_1_1StateObserverParam.html</anchorfile>
      <anchor>a63182d73f65f30e53a4d16f8000d2f65</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>Eigen::MatrixXd</type>
      <name>get_C</name>
      <anchorfile>classstate__observer_1_1StateObserverParam.html</anchorfile>
      <anchor>a64b622ec589ebcd67d7f19785bc8451b</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>Eigen::MatrixXd</type>
      <name>get_D</name>
      <anchorfile>classstate__observer_1_1StateObserverParam.html</anchorfile>
      <anchor>a8446729889c05c151c6194eb2b3e3fd3</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual std::string</type>
      <name>get_type</name>
      <anchorfile>classstate__observer_1_1StateObserverParam.html</anchorfile>
      <anchor>a4523494c23ad65f18ff3c4e7acd0f000</anchor>
      <arglist>() const</arglist>
    </member>
  </compound>
</tagfile>
